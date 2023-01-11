# MIT License
#
# Copyright (c) 2022 Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill
# Stachniss.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
from dataclasses import dataclass
from typing import Optional

from rich import box
from rich.console import Console
from rich.table import Table


class PipelineResults:
    def __init__(self) -> None:
        self._results = []

    def empty(self) -> bool:
        return len(self._results) == 0

    def print(self) -> None:
        # TODO(Nacho): Add mechanism to print ASCII if rich is not available
        self.log_to_console()

    def append(self, desc: str, units: str, value: float, trunc: bool = False):
        @dataclass
        class Metric:
            desc: str
            units: str
            value: float

        self._results.append(Metric(desc, units, int(value) if trunc else value))

    def log_to_file(self, filename: str, title: Optional[str]) -> None:
        with open(filename, "wt") as logfile:
            console = Console(file=logfile, width=100, force_jupyter=False)
            if title:
                console.rule(title)
            console.print(self._rich_table(table_format=box.ASCII_DOUBLE_HEAD))

    def log_to_console(self) -> None:
        if self.empty():
            return
        console = Console()
        console.print(self._rich_table())

    def _rich_table(self, table_format: box.Box = box.HORIZONTALS) -> Table:
        """Takes a metrics dictionary and sptis a Rich Table with the experiment results"""
        table = Table(box=table_format)
        table.add_column("Metric", justify="right", style="cyan")
        table.add_column("Value", justify="center", style="magenta")
        table.add_column("Units", justify="left", style="green")
        for result in self._results:
            table.add_row(
                result.desc,
                f"{result.value:{'.3f' if isinstance(result.value, float) else ''}}",
                result.units,
            )
        return table

    def __iter__(self):
        return self._results.__iter__()
