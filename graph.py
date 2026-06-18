# graph.py — Lightweight scrolling graph using Tkinter Canvas

from collections import deque


class ScrollingGraph:
    """
    Draws a scrolling line graph on a tk.Canvas widget.
    No matplotlib required — pure Tkinter.

    Optionally overlays a second series (its own min/max/color/unit) on the
    same canvas, scaled independently so two signals with different ranges
    (e.g. sccm and volts) can share one plot without one flattening the other.
    """

    def __init__(self, canvas, maxlen=60,
                 min_val=0.0, max_val=1.0,
                 line_color="lime", unit="",
                 secondary_min=None, secondary_max=None,
                 secondary_color="yellow", secondary_unit=""):
        self.canvas     = canvas
        self.min_val    = min_val
        self.max_val    = max_val
        self.line_color = line_color
        self.unit       = unit
        self.data       = deque([0.0] * maxlen, maxlen=maxlen)

        # Secondary (overlay) series — active only if both bounds are given.
        self.has_secondary   = secondary_min is not None and secondary_max is not None
        self.secondary_min   = secondary_min
        self.secondary_max   = secondary_max
        self.secondary_color = secondary_color
        self.secondary_unit  = secondary_unit
        self.secondary_data  = deque([0.0] * maxlen, maxlen=maxlen) if self.has_secondary else None

    def push(self, value, secondary_value=None):
        self.data.append(float(value))
        if self.has_secondary:
            self.secondary_data.append(float(secondary_value) if secondary_value is not None else 0.0)

    @staticmethod
    def _scaled_points(data, min_val, max_val, w, h):
        val_range = (max_val - min_val) or 1.0
        n = len(data)
        pts = []
        for i, val in enumerate(data):
            x = int(i * (w - 1) / (n - 1))
            y = int(h - (val - min_val) / val_range * h)
            y = max(0, min(h - 1, y))
            pts.append((x, y))
        return pts

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

        # ── Horizontal grid lines (quarters, primary scale) ──
        val_range = self.max_val - self.min_val or 1.0
        for i in range(1, 4):
            y     = int(h * i / 4)
            v_lbl = self.max_val - (val_range * i / 4)
            c.create_line(0, y, w, y, fill="#333333")
            c.create_text(3, y - 1, anchor="sw",
                          text=f"{v_lbl:.1f}", fill="#555555",
                          font=("Courier", 7))

        n = len(self.data)
        if n < 2:
            return

        # ── Primary data line ────────────────────────
        pts = self._scaled_points(self.data, self.min_val, self.max_val, w, h)
        for i in range(1, len(pts)):
            c.create_line(pts[i-1][0], pts[i-1][1],
                          pts[i][0],   pts[i][1],
                          fill=self.line_color, width=2)

        # ── Secondary (overlay) data line ────────────
        if self.has_secondary and len(self.secondary_data) >= 2:
            pts2 = self._scaled_points(self.secondary_data, self.secondary_min,
                                       self.secondary_max, w, h)
            for i in range(1, len(pts2)):
                c.create_line(pts2[i-1][0], pts2[i-1][1],
                              pts2[i][0],   pts2[i][1],
                              fill=self.secondary_color, width=1, dash=(4, 2))

        # ── Current value labels ─────────────────────
        last = self.data[-1]
        c.create_text(w - 4, 4, anchor="ne",
                      text=f"{last:.3f} {self.unit}",
                      fill="white", font=("Courier", 9, "bold"))

        if self.has_secondary:
            last2 = self.secondary_data[-1]
            c.create_text(4, 4, anchor="nw",
                          text=f"{last2:.3f} {self.secondary_unit}",
                          fill=self.secondary_color, font=("Courier", 9, "bold"))
