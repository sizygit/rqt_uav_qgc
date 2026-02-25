"""
drone_controller.py
-------------------
Manages SSH connection and remote process lifecycle for a single UAV,
using **paramiko** for all SSH operations.

Design:
  - paramiko.SSHClient is used for a persistent SSH session.
  - Agent is launched via an SSH channel with a pty; stdout is streamed
    to output_queue so the GUI can display it without blocking.
  - sudo passwords are fed via stdin over the same channel.
  - output_queue (thread-safe Queue) is polled by the GUI at 200 ms cadence.
"""

import queue
import threading
from typing import Optional

try:
    import paramiko
    _PARAMIKO_OK = True
except ImportError:
    _PARAMIKO_OK = False


class sshDroneManager:
    """Controls a single drone over SSH using paramiko."""

    XRCE_AGENT_CMD = 'MicroXRCEAgent udp4 -p {port}'
    XRCE_PGREP_CMD = 'pgrep -f "MicroXRCEAgent udp4"'
    XRCE_PKILL_CMD = 'pkill -f "MicroXRCEAgent udp4"'

    def __init__(
        self,
        drone_id: str,
        ip: str,
        username: str,
        password: Optional[str] = None,
        ssh_port: int = 22,
    ):
        if not _PARAMIKO_OK:
            raise ImportError(
                "paramiko is not installed.  Run:\n"
                "  pip install paramiko\n"
                "or:  sudo apt install python3-paramiko"
            )

        self.drone_id = drone_id
        self.ip       = ip
        self.username = username
        self.password = password
        self.ssh_port = ssh_port

        # Thread-safe message queue polled by the GUI.
        self.output_queue: queue.Queue = queue.Queue()

        self._ssh: Optional[paramiko.SSHClient] = None
        self._is_connected: bool = False

        # Agent state
        self._agent_channel:  Optional[object] = None  # paramiko.Channel
        self._agent_running:  bool = False
        self._agent_stop_evt: threading.Event = threading.Event()

    # ------------------------------------------------------------------ #
    #  Properties
    # ------------------------------------------------------------------ #

    @property
    def is_connected(self) -> bool:
        return self._is_connected and self._ssh is not None

    @property
    def is_agent_running(self) -> bool:
        return self._agent_running

    # ------------------------------------------------------------------ #
    #  Public API
    # ------------------------------------------------------------------ #

    def connect(self) -> bool:
        """Open a persistent SSH session.  Returns True on success."""
        if self.is_connected:
            self._put('Already connected.')
            return True

        self._put(f'Connecting to {self.username}@{self.ip}:{self.ssh_port}...')

        client = paramiko.SSHClient()
        client.set_missing_host_key_policy(paramiko.AutoAddPolicy())

        try:
            client.connect(
                hostname=self.ip,
                port=self.ssh_port,
                username=self.username,
                password=self.password,
                timeout=10,
                auth_timeout=10,
                banner_timeout=10,
                allow_agent=False,
                look_for_keys=False,
            )
        except paramiko.AuthenticationException:
            self._put('Authentication failed – wrong password?')
            return False
        except paramiko.SSHException as exc:
            self._put(f'SSH error: {exc}')
            return False
        except OSError as exc:
            self._put(f'Network error: {exc}')
            return False
        except Exception as exc:
            self._put(f'Unexpected error: {exc}')
            return False

        self._ssh = client
        self._is_connected = True
        self._put('SSH connection established.')
        return True

    def disconnect(self):
        """Close the SSH session and clean up."""
        if self.is_agent_running:
            self.stop_agent()
        if self._ssh:
            try:
                self._ssh.close()
            except Exception:
                pass
            self._ssh = None
        self._is_connected = False
        self._put('Disconnected.')

    def start_agent(self, port: int = 8888) -> bool:
        """
        Launch MicroXRCEAgent on the remote host.
        Streams stdout/stderr to output_queue via a daemon thread.
        """
        if not self.is_connected:
            self._put('Not connected – cannot start agent.')
            return False

        if self._agent_running:
            self._put('Agent already running.')
            return True

        # Check whether another instance is already running remotely.
        out, _ = self._exec_sync(self.XRCE_PGREP_CMD, timeout=5)
        if out and out.strip():
            self._put(
                f'MicroXRCEAgent already running on remote '
                f'(pid: {out.strip()}).  Kill it first.'
            )
            self._agent_running = True
            return True

        cmd = self.XRCE_AGENT_CMD.format(port=port)
        self._put(f'Launching: {cmd}')

        try:
            transport = self._ssh.get_transport()
            channel   = transport.open_session()
            channel.get_pty()
            channel.exec_command(cmd)
        except Exception as exc:
            self._put(f'Failed to open channel: {exc}')
            return False

        self._agent_channel  = channel
        self._agent_running  = True
        self._agent_stop_evt.clear()

        threading.Thread(
            target=self._stream_channel,
            args=(channel,),
            daemon=True,
        ).start()

        return True

    def stop_agent(self) -> bool:
        """Kill MicroXRCEAgent on the remote host."""
        if not self.is_connected:
            self._put('Not connected.')
            return False

        self._agent_stop_evt.set()

        if self._agent_channel:
            try:
                self._agent_channel.close()
            except Exception:
                pass
            self._agent_channel = None

        # pkill just in case the process survived channel close.
        self._exec_sync(self.XRCE_PKILL_CMD, timeout=8)
        self._put('Agent stopped.')
        self._agent_running = False
        return True

    def shutdown_remote(self) -> bool:
        """Execute `sudo shutdown -h now` on the remote host."""
        if not self.is_connected:
            self._put('Not connected.')
            return False

        self._put(f'Sending shutdown to {self.ip}...')

        try:
            transport = self._ssh.get_transport()
            channel   = transport.open_session()
            channel.get_pty()
            channel.exec_command('sudo shutdown -h now')

            # Feed the sudo password via stdin (works when sudoers requires pw).
            if self.password:
                channel.sendall((self.password + '\n').encode())

            # Drain output briefly.
            output = b''
            channel.settimeout(5)
            try:
                while True:
                    chunk = channel.recv(1024)
                    if not chunk:
                        break
                    output += chunk
            except Exception:
                pass

            if output:
                self._put(output.decode(errors='replace').strip())
            else:
                self._put('Shutdown command accepted.')

            channel.close()
        except Exception as exc:
            self._put(f'Shutdown error: {exc}')
            return False

        self._is_connected = False
        return True

    def cleanup(self):
        """Called when the plugin closes."""
        self.disconnect()

    # ------------------------------------------------------------------ #
    #  Internal helpers
    # ------------------------------------------------------------------ #

    def _put(self, msg: str):
        """Thread-safe message push."""
        self.output_queue.put(f'[{self.drone_id}] {msg}')

    def _exec_sync(self, cmd: str, timeout: int = 8):
        """Run a one-shot command, return (stdout_str, stderr_str)."""
        if not self._ssh:
            return '', ''
        try:
            _, stdout, stderr = self._ssh.exec_command(cmd, timeout=timeout)
            out = stdout.read().decode(errors='replace')
            err = stderr.read().decode(errors='replace')
            return out, err
        except Exception as exc:
            self._put(f'exec error ({cmd!r}): {exc}')
            return '', ''

    def _stream_channel(self, channel):
        """
        Read output from the SSH channel and push lines to output_queue.
        Runs in a daemon thread.  Exits when the channel closes or
        _agent_stop_evt is set.
        """
        channel.settimeout(1.0)
        buf = b''
        while not self._agent_stop_evt.is_set():
            try:
                chunk = channel.recv(4096)
                if not chunk:
                    break          # EOF: remote process exited
                buf += chunk
                # Flush complete lines.
                while b'\n' in buf:
                    line, buf = buf.split(b'\n', 1)
                    txt = line.decode(errors='replace').rstrip()
                    if txt:
                        self.output_queue.put(f'[{self.drone_id}:agent] {txt}')
            except Exception:
                # Timeout or interrupt – keep looping until stop_evt is set.
                continue

        # Flush remainder.
        if buf:
            txt = buf.decode(errors='replace').rstrip()
            if txt:
                self.output_queue.put(f'[{self.drone_id}:agent] {txt}')

        self._agent_running = False
        self._put('Agent process exited.')
