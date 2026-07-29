# graph.py — Lightweight scrolling graph using Tkinter Canvas

from collections import deque


class ScrollingGraph:
    """
    Draws a scrolling line graph on a tk.Canvas widget.
    No matplotlib required — pure Tkinter.

    Optionally overlays a second series (its own min/max/color/unit) on the
    same canvas, scaled independently so two signals with different ranges
    (e.g. sccm and volts) can share one plot without one flattening the other.

    Performance note: canvas items (background, grid, per-sample line
    segments, legend) are created ONCE and repositioned in place on every
    draw() call via canvas.coords()/itemconfigure(), instead of being
    delete("all")'d and rebuilt from scratch each time. `self.data` (and
    each series' data) is a fixed-maxlen deque, so the number of line
    segments never changes after construction — only their endpoints do —
    which is what makes reuse safe. Rebuilding ~500 canvas primitives every
    GUI_REFRESH_INTERVAL (200ms) across four graphs was measurably laggy on
    the Pi 2B once the 5-series PZEM graph was added; canvas.coords() on
    existing items is far cheaper than full recreation.
    """

    def __init__(self, canvas, maxlen=60,
                 min_val=0.0, max_val=1.0,
                 line_color="#00FF00", unit="",  # lime, as a hex code -- see
                                                 # config.py's GRAPH_*_COLOR comment
                 secondary_min=None, secondary_max=None,
                 secondary_color="#FFFF00", secondary_unit="",  # yellow
                 series=None):
        self.canvas     = canvas
        self.maxlen     = maxlen
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

        # Named multi-series mode (arbitrary count, drawn with a legend) —
        # only active when `series` is given; leaves the primary/secondary
        # path above completely untouched for every existing caller.
        # series: list of {"name", "color", "min", "max", "unit"(optional)}
        self.series = None
        if series is not None:
            self.series = [
                {
                    "name":  s["name"],
                    "color": s["color"],
                    "min":   s["min"],
                    "max":   s["max"],
                    "unit":  s.get("unit", ""),
                    "data":  deque([0.0] * maxlen, maxlen=maxlen),
                }
                for s in series
            ]

        # Persistent canvas item handles, populated by _build()/_build_multi()
        # on the first draw() (or after a resize) and reused thereafter.
        self._built     = False
        self._last_size = (0, 0)

    def push(self, value, secondary_value=None):
        self.data.append(float(value))
        if self.has_secondary:
            self.secondary_data.append(float(secondary_value) if secondary_value is not None else 0.0)

    def push_series(self, values):
        """Multi-series mode only. `values` maps series name -> reading."""
        for s in self.series:
            s["data"].append(float(values.get(s["name"], 0.0)))

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

        resized = (w, h) != self._last_size

        if self.series is not None:
            if not self._built or resized:
                self._build_multi(c, w, h)
                self._last_size = (w, h)
            else:
                self._update_multi(w, h)
            return

        if not self._built or resized:
            self._build(c, w, h)
            self._last_size = (w, h)
        else:
            self._update(w, h)

    # ────────────────────────────────────────────────
    #  Single/dual-series mode
    # ────────────────────────────────────────────────

    def _build(self, c, w, h):
        c.delete("all")

        # ── Background ──────────────────────────────
        c.create_rectangle(0, 0, w, h, fill="#1a1a1a", outline="")

        # ── Horizontal grid lines (quarters, primary scale) — static,
        # never repositioned after this (only rebuilt on resize) ──
        val_range = self.max_val - self.min_val or 1.0
        for i in range(1, 4):
            y     = int(h * i / 4)
            v_lbl = self.max_val - (val_range * i / 4)
            c.create_line(0, y, w, y, fill="#333333")
            c.create_text(3, y - 1, anchor="sw",
                          text=f"{v_lbl:.1f}", fill="#555555",
                          font=("Courier", 9))

        # ── Primary data line: one item per segment, fixed count for the
        # life of the graph (self.data is a fixed-maxlen deque) ──
        pts = self._scaled_points(self.data, self.min_val, self.max_val, w, h)
        self._prim_segs = [
            c.create_line(pts[i-1][0], pts[i-1][1], pts[i][0], pts[i][1],
                          fill=self.line_color, width=2)
            for i in range(1, len(pts))
        ]

        # ── Secondary (overlay) data line ────────────
        self._sec_segs = []
        if self.has_secondary:
            pts2 = self._scaled_points(self.secondary_data, self.secondary_min,
                                       self.secondary_max, w, h)
            self._sec_segs = [
                c.create_line(pts2[i-1][0], pts2[i-1][1], pts2[i][0], pts2[i][1],
                              fill=self.secondary_color, width=1, dash=(4, 2))
                for i in range(1, len(pts2))
            ]

        # ── Current value labels ─────────────────────
        last = self.data[-1]
        self._val_text = c.create_text(
            w - 4, 4, anchor="ne", text=f"{last:.3f} {self.unit}",
            fill="white", font=("Courier", 11, "bold"))

        self._val_text2 = None
        if self.has_secondary:
            last2 = self.secondary_data[-1]
            self._val_text2 = c.create_text(
                4, 4, anchor="nw", text=f"{last2:.3f} {self.secondary_unit}",
                fill=self.secondary_color, font=("Courier", 11, "bold"))

        self._built = True

    def _update(self, w, h):
        c = self.canvas

        pts = self._scaled_points(self.data, self.min_val, self.max_val, w, h)
        for i, item_id in enumerate(self._prim_segs, start=1):
            c.coords(item_id, pts[i-1][0], pts[i-1][1], pts[i][0], pts[i][1])
        c.itemconfigure(self._val_text, text=f"{self.data[-1]:.3f} {self.unit}")

        if self.has_secondary:
            pts2 = self._scaled_points(self.secondary_data, self.secondary_min,
                                       self.secondary_max, w, h)
            for i, item_id in enumerate(self._sec_segs, start=1):
                c.coords(item_id, pts2[i-1][0], pts2[i-1][1], pts2[i][0], pts2[i][1])
            c.itemconfigure(self._val_text2,
                            text=f"{self.secondary_data[-1]:.3f} {self.secondary_unit}")

    # ────────────────────────────────────────────────
    #  Named multi-series mode
    # ────────────────────────────────────────────────

    def _build_multi(self, c, w, h):
        """Named multi-series mode: N independently-scaled overlaid lines
        plus a legend row (color swatch + name + latest value) reserved at
        the top of the canvas. Each series is scaled within its own
        min/max, same independent-scaling approach as the primary/secondary
        pair above, generalized to an arbitrary count.
        """
        c.delete("all")
        c.create_rectangle(0, 0, w, h, fill="#1a1a1a", outline="")

        self._legend_h = 18
        plot_top = self._legend_h
        plot_h = max(1, h - plot_top)

        # ── Legend row ────────────────────────────────
        x = 4
        self._legend_swatches = []
        self._legend_texts = []
        for s in self.series:
            last = s["data"][-1] if s["data"] else 0.0
            label = f"{s['name']}: {last:.2f}{s['unit']}"
            swatch = c.create_rectangle(x, 5, x + 9, 14, fill=s["color"], outline="")
            text_id = c.create_text(x + 12, 3, anchor="nw", text=label,
                                    fill=s["color"], font=("Courier", 9, "bold"))
            self._legend_swatches.append(swatch)
            self._legend_texts.append(text_id)
            bbox = c.bbox(text_id)
            x = (bbox[2] if bbox else x + 10) + 10

        # ── Grid lines (quarters, within the plot area) — static ──
        for i in range(1, 4):
            y = plot_top + int(plot_h * i / 4)
            c.create_line(0, y, w, y, fill="#333333")

        # ── Per-series data lines: fixed segment count per series (each
        # series' data is a fixed-maxlen deque) ──
        self._series_segs = []
        for s in self.series:
            pts = self._scaled_points(s["data"], s["min"], s["max"], w, plot_h)
            pts = [(px, py + plot_top) for px, py in pts]
            segs = [
                c.create_line(pts[i-1][0], pts[i-1][1], pts[i][0], pts[i][1],
                              fill=s["color"], width=2)
                for i in range(1, len(pts))
            ]
            self._series_segs.append(segs)

        self._built = True

    def _update_multi(self, w, h):
        c = self.canvas
        plot_top = self._legend_h
        plot_h = max(1, h - plot_top)

        # Legend swatches stay put (fixed layout order); only the text
        # (and thus its width) can change tick to tick, so swatch position
        # is left as-built rather than re-measured every frame — cheap and
        # visually stable since series order/labels never change shape much.
        for s, text_id in zip(self.series, self._legend_texts):
            last = s["data"][-1] if s["data"] else 0.0
            c.itemconfigure(text_id, text=f"{s['name']}: {last:.2f}{s['unit']}")

        for s, segs in zip(self.series, self._series_segs):
            pts = self._scaled_points(s["data"], s["min"], s["max"], w, plot_h)
            pts = [(px, py + plot_top) for px, py in pts]
            for i, item_id in enumerate(segs, start=1):
                c.coords(item_id, pts[i-1][0], pts[i-1][1], pts[i][0], pts[i][1])
