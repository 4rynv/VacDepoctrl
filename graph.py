# graph.py — Lightweight scrolling graph using Tkinter Canvas

from collections import deque


class ScrollingGraph:
    """
    Draws a scrolling line graph on a tk.Canvas widget.
    No matplotlib required — pure Tkinter.
    """

    def __init__(self, canvas, maxlen=60,
                 min_val=0.0, max_val=1.0,
                 line_color="lime", unit=""):
        self.canvas     = canvas
        self.min_val    = min_val
        self.max_val    = max_val
        self.line_color = line_color
        self.unit       = unit
        self.data       = deque([0.0] * maxlen, maxlen=maxlen)

    def push(self, value):
        self.data.append(float(value))

    def draw(self):
        c = self.canvas
        w = c.winfo_width()
        h = c.winfo_height()

        # Canvas not yet rendered
        if w <= 1 or h <= 1:
            return

        c.delete("all")

        # ── Background ──────────────────────────────
        c.create_rectangle(0, 0, w, h, fill="#1a1a1a", outline="")

        # ── Horizontal grid lines (quarters) ────────
        val_range = self.max_val - self.min_val or 1.0
        for i in range(1, 4):
            y     = int(h * i / 4)
            v_lbl = self.max_val - (val_range * i / 4)
            c.create_line(0, y, w, y, fill="#333333")
            c.create_text(3, y - 1, anchor="sw",
                          text=f"{v_lbl:.1f}", fill="#555555",
                          font=("Courier", 7))

        # ── Data line ────────────────────────────────
        n = len(self.data)
        if n < 2:
            return

        pts = []
        for i, val in enumerate(self.data):
            x = int(i * (w - 1) / (n - 1))
            y = int(h - (val - self.min_val) / val_range * h)
            y = max(0, min(h - 1, y))
            pts.append((x, y))

        for i in range(1, len(pts)):
            c.create_line(pts[i-1][0], pts[i-1][1],
                          pts[i][0],   pts[i][1],
                          fill=self.line_color, width=2)

        # ── Current value (top-right) ─────────────
        last = self.data[-1]
        c.create_text(w - 4, 4, anchor="ne",
                      text=f"{last:.3f} {self.unit}",
                      fill="white", font=("Courier", 9, "bold"))
