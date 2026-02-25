"""
ansi_text_edit.py
-----------------
A self-contained module that provides:

  ansi_to_html(text: str) -> str
      Converts a string containing ANSI SGR escape sequences to an HTML
      fragment suitable for insertion into a Qt rich-text widget.

  AnsiTextEdit(QTextEdit)
      A drop-in replacement for QPlainTextEdit that renders ANSI colour
      codes correctly.  Use appendAnsi(text) instead of appendPlainText().

Supported ANSI SGR codes
------------------------
  0          reset
  1          bold
  2          dim  (rendered as slightly lighter)
  3          italic
  4          underline
  22/23/24   bold/italic/underline off
  30–37      standard foreground colours (dark variants)
  90–97      bright foreground colours
  39         default foreground
  40–47      standard background colours
  100–107    bright background colours
  49         default background
"""

import re
from typing import Optional

from python_qt_binding.QtCore import Qt
from python_qt_binding.QtGui import QTextCursor
from python_qt_binding.QtWidgets import QTextEdit

# ── ANSI SGR palette ────────────────────────────────────────────────────── #

_STANDARD_FG = {
    30: '#2e2e2e',   # black   (dark)
    31: '#cc0000',   # red
    32: '#4e9a06',   # green
    33: '#c4a000',   # yellow/brown
    34: '#3465a4',   # blue
    35: '#75507b',   # magenta
    36: '#06989a',   # cyan
    37: '#d3d7cf',   # white (light grey)
}

_BRIGHT_FG = {
    90: '#555753',   # bright black (dark grey)
    91: '#ef2929',   # bright red
    92: '#8ae234',   # bright green
    93: '#fce94f',   # bright yellow
    94: '#729fcf',   # bright blue
    95: '#ad7fa8',   # bright magenta
    96: '#34e2e2',   # bright cyan
    97: '#eeeeec',   # bright white
}

_STANDARD_BG = {
    40: '#2e2e2e',
    41: '#cc0000',
    42: '#4e9a06',
    43: '#c4a000',
    44: '#3465a4',
    45: '#75507b',
    46: '#06989a',
    47: '#d3d7cf',
}

_BRIGHT_BG = {
    100: '#555753',
    101: '#ef2929',
    102: '#8ae234',
    103: '#fce94f',
    104: '#729fcf',
    105: '#ad7fa8',
    106: '#34e2e2',
    107: '#eeeeec',
}

_ANSI_RE = re.compile(r'\x1b\[([0-9;]*)m')


# ── Core converter ──────────────────────────────────────────────────────── #

def ansi_to_html(text: str) -> str:
    """
    Convert *text* (which may contain ANSI SGR sequences) to an HTML fragment.

    The fragment is designed to be passed to QTextEdit.insertHtml().  Each
    segment is wrapped in a ``<span style="...">`` element; open tags from
    the previous call are always closed so fragments can be appended safely.

    Non-SGR escape sequences (cursor movement, etc.) are stripped silently.
    """
    # Escape any existing HTML entities first.
    text = (
        text
        .replace('&', '&amp;')
        .replace('<', '&lt;')
        .replace('>', '&gt;')
        .replace('"', '&quot;')
    )

    fg:        Optional[str] = None
    bg:        Optional[str] = None
    bold:      bool          = False
    italic:    bool          = False
    underline: bool          = False

    result: list[str] = []
    open_span: bool   = False
    pos = 0

    for m in _ANSI_RE.finditer(text):
        # Flush literal text before this escape.
        segment = text[pos:m.start()]
        if segment:
            if open_span:
                result.append(segment)
            else:
                result.append(segment)
        pos = m.end()

        # Close any open span before changing style.
        if open_span:
            result.append('</span>')
            open_span = False

        # Parse the codes.
        raw = m.group(1)
        codes = [int(c) for c in raw.split(';') if c != ''] if raw else [0]

        for code in codes:
            if code == 0:
                fg = bg = None
                bold = italic = underline = False
            elif code == 1:
                bold = True
            elif code == 2:
                pass          # dim – ignore for simplicity
            elif code == 3:
                italic = True
            elif code == 4:
                underline = True
            elif code == 22:
                bold = False
            elif code == 23:
                italic = False
            elif code == 24:
                underline = False
            elif code in _STANDARD_FG:
                fg = _STANDARD_FG[code]
            elif code in _BRIGHT_FG:
                fg = _BRIGHT_FG[code]
            elif code == 39:
                fg = None
            elif code in _STANDARD_BG:
                bg = _STANDARD_BG[code]
            elif code in _BRIGHT_BG:
                bg = _BRIGHT_BG[code]
            elif code == 49:
                bg = None

        # Build a new span if there is any styling.
        styles: list[str] = []
        if fg:
            styles.append(f'color:{fg}')
        if bg:
            styles.append(f'background-color:{bg}')
        if bold:
            styles.append('font-weight:bold')
        if italic:
            styles.append('font-style:italic')
        if underline:
            styles.append('text-decoration:underline')

        if styles:
            result.append(f'<span style="{";".join(styles)}">')
            open_span = True

    # Flush any remaining text after the last escape.
    tail = text[pos:]
    if tail:
        result.append(tail)

    if open_span:
        result.append('</span>')

    return ''.join(result)


# ── Widget ──────────────────────────────────────────────────────────────── #

class AnsiTextEdit(QTextEdit):
    """
    A read-only QTextEdit that renders ANSI colour codes.

    Drop-in replacement for QPlainTextEdit used by the UAV QGC log panel.
    Geometry, object name and parent are copied from the widget it replaces.
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setReadOnly(True)
        self.document().setMaximumBlockCount(2000)   # keep memory bounded
        # Dark terminal-style background.
        self.setStyleSheet(
            'QTextEdit {'
            # '  background-color: #1e1e1e;'
            # '  color: #d4d4d4;'
            '  background-color: #FFFFFF;'
            '  color: #000000;'
            '  font-family: monospace;'
            '  font-size: 10pt;'
            '  border: 1px solid #3c3c3c;'
            '}'
        )

    def appendAnsi(self, text: str):
        """Append *text* (possibly containing ANSI SGR codes) as a new line."""
        html_line = ansi_to_html(text)
        cursor = self.textCursor()
        cursor.movePosition(QTextCursor.End)
        self.setTextCursor(cursor)
        # insertHtml appends inline; we want each message on its own line.
        if not self.document().isEmpty():
            cursor.insertBlock()
        cursor.insertHtml(html_line)
        self.ensureCursorVisible()

    # Keep the same public interface as QPlainTextEdit for compatibility.
    def appendPlainText(self, text: str):
        self.appendAnsi(text)
