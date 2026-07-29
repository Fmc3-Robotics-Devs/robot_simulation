"""The outfeed tray's slots (plan section 9.5).

Fixed grid, occupancy kept in memory: enough for a cell that is emptied by
hand. The offsets are relative to the tray's surveyed drop pose, so the actual
world position still comes from the tag observation at placement time - the
book only answers "which free slot next", never "where is the tray".
"""


class TrayFull(RuntimeError):
    pass


class SlotBook:
    def __init__(self, rows=1, columns=2, pitch=(0.12, 0.12)):
        if rows < 1 or columns < 1:
            raise ValueError("a tray needs at least one slot")
        self._rows = rows
        self._columns = columns
        self._pitch = pitch
        self._occupied = set()

    def _names(self):
        return [
            f"{row}_{column}"
            for row in range(self._rows)
            for column in range(self._columns)
        ]

    def offset(self, name):
        """Slot offset from the tray's drop pose, centred on the grid."""
        row_text, column_text = name.split("_")
        row, column = int(row_text), int(column_text)
        if not (0 <= row < self._rows and 0 <= column < self._columns):
            raise KeyError(f"no slot '{name}' in a {self._rows}x{self._columns} tray")
        return (
            (row - (self._rows - 1) / 2.0) * self._pitch[0],
            (column - (self._columns - 1) / 2.0) * self._pitch[1],
        )

    def claim(self, name=""):
        """Reserve a slot and return (name, offset). Empty name picks the next free."""
        if name:
            if name in self._occupied:
                raise TrayFull(f"slot '{name}' is already occupied")
            self._occupied.add(name)
            return name, self.offset(name)

        for candidate in self._names():
            if candidate not in self._occupied:
                self._occupied.add(candidate)
                return candidate, self.offset(candidate)
        raise TrayFull(f"all {self._rows * self._columns} slots are occupied")

    def release(self, name):
        self._occupied.discard(name)

    def reset(self):
        self._occupied.clear()

    @property
    def free(self):
        return [name for name in self._names() if name not in self._occupied]
