#!/usr/bin/env python3
"""
merge_latency_runs.py

Merges latency CSV data into an Excel file.

Excel has a maximum of 1,048,576 rows per worksheet, so large
datasets are automatically split across multiple raw-data sheets.

Usage:
    python3 merge_latency_runs.py
    python3 merge_latency_runs.py --input ~/latency_data --output ~/latency_results.xlsx
"""

import argparse
import os
import glob
import pandas as pd
from openpyxl import load_workbook
from openpyxl.styles import Font, PatternFill, Alignment, Border, Side
from openpyxl.utils import get_column_letter
from openpyxl.chart import LineChart, Reference, BarChart
from pathlib import Path
import matplotlib.pyplot as plt
from openpyxl import Workbook


# Excel maximum worksheet size is 1,048,576 rows.
# One row is used for the header, so leave 1,048,575 rows for data.
EXCEL_MAX_ROWS = 1_048_576
MAX_DATA_ROWS_PER_SHEET = EXCEL_MAX_ROWS - 1


def load_runs(input_dir, period):
    """Load CSV data."""

    df = pd.read_csv(input_dir)

    df = df.rename(columns={'control loop iteration': 'iteration'})
    df = df.rename(columns={'iteration_difference_ms': 'period_ms'})

    if "iteration" in df.columns:
        df = df.drop(columns=["iteration"])

    # Remove first 10000 samples
    if period == 0.01:
        df = df[10000:310000]
    else:
        df = df[1000:31000]
    return df


def build_dataframe(runs):
    """Build combined DataFrame — one column per run, rows are frame indices."""

    df = pd.DataFrame(runs)
    df.index.name = "frame"

    return df


def remove_outliers(series):
    """Remove outliers using IQR method."""

    s = series.dropna()

    Q1 = s.quantile(0.25)
    Q3 = s.quantile(0.75)
    IQR = Q3 - Q1

    lower_bound = Q1 - 1.5 * IQR
    upper_bound = Q3 + 1.5 * IQR

    filtered = s[(s >= lower_bound) & (s <= upper_bound)]

    return filtered

def missed_deadlines(runs_df, period):
    misses = 0
    print("Period: ", period)

    for i, row in runs_df.iterrows():
        if row['period_ms'] >= 2*period:
            misses += 1

    return misses

def add_stats_sheet(wb, runs_df, period):
    """Add a summary statistics sheet."""

    ws = wb.create_sheet("Statistics")

    headers = [
        "Run",
        "Frames",
        "Mean (ms)",
        "Median (ms)",
        "P(95)",
        "P(99)",
        "Min (ms)",
        "Max (ms)",
        "Std Dev (ms)",
        "Missed Deadlines"
    ]

    # Header row
    for col, h in enumerate(headers, 1):
        cell = ws.cell(row=1, column=col, value=h)
        cell.font = Font(
            name="Arial",
            bold=True,
            color="FFFFFF"
        )
        cell.fill = PatternFill(
            "solid",
            fgColor="1F4E79"
        )
        cell.alignment = Alignment(
            horizontal="center"
        )

    # Data rows
    for row, run_name in enumerate(runs_df.columns, 2):

        s = runs_df[run_name]

        ws.cell(
            row=row,
            column=1,
            value=run_name
        ).font = Font(
            name="Arial",
            bold=True
        )

        ws.cell(
            row=row,
            column=2,
            value=len(s)
        )

        ws.cell(
            row=row,
            column=3,
            value=round(s.mean(), 6)
        )

        ws.cell(
            row=row,
            column=4,
            value=round(s.median(), 6)
        )

        ws.cell(
            row=row,
            column=5,
            value=round(s.quantile(0.95), 6)
        )

        ws.cell(
            row=row,
            column=6,
            value=round(s.quantile(0.99), 6)
        )

        ws.cell(
            row=row,
            column=7,
            value=round(s.min(), 6)
        )

        ws.cell(
            row=row,
            column=8,
            value=round(s.max(), 6)
        )

        ws.cell(
            row=row,
            column=9,
            value=round(s.std(), 6)
        )

        ws.cell(
            row=row,
            column=10,
            value=missed_deadlines(runs_df=runs_df, period=period)
        )

        # Alternate row shading
        fill_color = (
            "D6E4F0"
            if row % 2 == 0
            else "FFFFFF"
        )

        for col in range(1, len(headers) + 1):

            c = ws.cell(
                row=row,
                column=col
            )

            c.fill = PatternFill(
                "solid",
                fgColor=fill_color
            )

            c.font = Font(name="Arial")

            c.alignment = Alignment(
                horizontal="center"
            )

    # Column widths
    col_widths = [
        40,
        10,
        12,
        14,
        10,
        10,
        12,
        14
    ]

    for col, width in enumerate(col_widths, 1):
        ws.column_dimensions[
            get_column_letter(col)
        ].width = width

    return ws


def add_raw_sheet(wb, runs_df):
    """
    Add raw latency data sheets.

    Excel can only contain 1,048,576 rows per worksheet.
    Since row 1 is the header, each sheet contains at most
    1,048,575 data rows.

    Sheets are named:
        Raw Latency (µs) 1
        Raw Latency (µs) 2
        ...
    """

    total_rows = len(runs_df)

    number_of_sheets = (
        (total_rows + MAX_DATA_ROWS_PER_SHEET - 1)
        // MAX_DATA_ROWS_PER_SHEET
    )

    print(
        f"[INFO] Raw data contains {total_rows} rows"
    )

    print(
        f"[INFO] Splitting raw data across "
        f"{number_of_sheets} Excel sheet(s)"
    )

    for sheet_number, start in enumerate(
        range(
            0,
            total_rows,
            MAX_DATA_ROWS_PER_SHEET
        ),
        1
    ):

        end = min(
            start + MAX_DATA_ROWS_PER_SHEET,
            total_rows
        )

        chunk = runs_df.iloc[start:end]

        if number_of_sheets == 1:
            sheet_name = "Raw Latency (µs)"
        else:
            sheet_name = f"Raw Latency (µs) {sheet_number}"

        print(
            f"[INFO] Writing {sheet_name}: "
            f"rows {start} - {end - 1}"
        )

        ws = wb.create_sheet(sheet_name)

        # Header
        frame_header = ws.cell(
            row=1,
            column=1,
            value="Frame"
        )

        frame_header.font = Font(
            name="Arial",
            bold=True,
            color="FFFFFF"
        )

        frame_header.fill = PatternFill(
            "solid",
            fgColor="1F4E79"
        )

        frame_header.alignment = Alignment(
            horizontal="center"
        )

        # Run headers
        for col, run_name in enumerate(
            chunk.columns,
            2
        ):

            cell = ws.cell(
                row=1,
                column=col,
                value=run_name
            )

            cell.font = Font(
                name="Arial",
                bold=True,
                color="FFFFFF"
            )

            cell.fill = PatternFill(
                "solid",
                fgColor="1F4E79"
            )

            cell.alignment = Alignment(
                horizontal="center"
            )

        # Data
        #
        # Use itertuples instead of repeated .loc access.
        # This is considerably faster for ~1 million rows.
        for row_offset, row_data in enumerate(
            chunk.itertuples(
                index=True,
                name=None
            ),
            2
        ):

            frame_idx = row_data[0]

            ws.cell(
                row=row_offset,
                column=1,
                value=frame_idx
            )

            for col_idx, value in enumerate(
                row_data[1:],
                2
            ):

                if pd.notna(value):

                    cell = ws.cell(
                        row=row_offset,
                        column=col_idx,
                        value=round(float(value), 2)
                    )

                    cell.font = Font(
                        name="Arial"
                    )

                    cell.alignment = Alignment(
                        horizontal="right"
                    )

        # Column widths
        ws.column_dimensions["A"].width = 8

        for col in range(
            2,
            len(chunk.columns) + 2
        ):

            ws.column_dimensions[
                get_column_letter(col)
            ].width = 35

    return number_of_sheets


def add_chart_sheet(wb, runs_df):
    """Add histogram charts of latency with 0.05 ms bin width."""

    ws = wb.create_sheet("Histogram")

    BIN_SIZE = 0.05  # milliseconds

    # Hidden worksheet containing histogram data
    data_ws = wb.create_sheet("Histogram Data")

    data_ws.cell(
        row=1,
        column=1,
        value="Latency (ms)"
    )

    # Find global min/max across all runs
    values = runs_df.stack().dropna()

    if values.empty:
        return ws

    min_value = values.min()
    max_value = values.max()

    # Align bins to multiples of 0.05
    bin_start = (
        int(min_value // BIN_SIZE)
        * BIN_SIZE
    )

    bin_end = (
        int(max_value // BIN_SIZE)
        + 1
    ) * BIN_SIZE

    # Generate bin edges
    bins = []

    current = bin_start

    while current <= bin_end:

        bins.append(
            round(current, 10)
        )

        current += BIN_SIZE

    # Bin labels = center of each bin
    bin_labels = [
        round(
            (bins[i] + bins[i + 1]) / 2,
            3
        )
        for i in range(len(bins) - 1)
    ]

    # Write bin labels
    for row_idx, label in enumerate(
        bin_labels,
        2
    ):

        data_ws.cell(
            row=row_idx,
            column=1,
            value=label
        )

    # Create histogram counts for each run
    for col_idx, run_name in enumerate(
        runs_df.columns,
        2
    ):

        data_ws.cell(
            row=1,
            column=col_idx,
            value=run_name
        )

        series = runs_df[
            run_name
        ].dropna()

        counts = []

        for i in range(
            len(bins) - 1
        ):

            lower = bins[i]
            upper = bins[i + 1]

            # Include lower bound, exclude upper bound
            count = (
                (series >= lower)
                & (series < upper)
            ).sum()

            counts.append(
                int(count)
            )

        for row_idx, count in enumerate(
            counts,
            2
        ):

            data_ws.cell(
                row=row_idx,
                column=col_idx,
                value=count
            )

    data_ws.sheet_state = "hidden"

    # Create chart
    chart = BarChart()

    chart.type = "col"
    chart.style = 10

    chart.title = "Latency Distribution"
    chart.y_axis.title = "Frequency"
    chart.x_axis.title = "Latency (ms)"

    chart.width = 30
    chart.height = 15

    max_rows = len(bin_labels) + 1

    # Add each run as a separate series
    for col_idx in range(
        2,
        len(runs_df.columns) + 2
    ):

        data_ref = Reference(
            data_ws,
            min_col=col_idx,
            min_row=1,
            max_row=max_rows
        )

        chart.add_data(
            data_ref,
            titles_from_data=True
        )

    # X-axis categories
    categories = Reference(
        data_ws,
        min_col=1,
        min_row=2,
        max_row=max_rows
    )

    chart.set_categories(categories)

    # Make bars touch
    chart.gapWidth = 0

    ws.add_chart(
        chart,
        "A1"
    )

    return ws


def add_max_chart_sheet(wb, runs_df):
    """Add separate raw line charts for each metric without binning."""

    metric_names = []

    for metric in ["execution_ms", "period_ms"]:
        if metric in runs_df.columns:
            metric_names.append(metric)

    if not metric_names:
        metric_names = [
            col for col in runs_df.columns
            if col not in ["iteration", "frame"]
        ]

    for metric_name in metric_names:

        ws = wb.create_sheet(f"{metric_name} Chart")

        data_ws = wb.create_sheet(f"{metric_name} Data")

        data_ws.cell(
            row=1,
            column=1,
            value="Frame"
        )

        data_ws.cell(
            row=1,
            column=2,
            value=metric_name
        )

        series = runs_df[metric_name].dropna()

        for row_idx, (frame_idx, value) in enumerate(
            series.items(),
            2
        ):
            data_ws.cell(
                row=row_idx,
                column=1,
                value=frame_idx
            )
            data_ws.cell(
                row=row_idx,
                column=2,
                value=float(value)
            )

        data_ws.sheet_state = "hidden"

        chart = LineChart()

        chart.title = f"{metric_name} per Frame"
        chart.style = 10

        chart.y_axis.title = metric_name
        chart.x_axis.title = "Frame"

        chart.width = 30
        chart.height = 15

        data_ref = Reference(
            data_ws,
            min_col=2,
            min_row=1,
            max_row=1 + len(series)
        )
        chart.add_data(
            data_ref,
            titles_from_data=True
        )

        categories = Reference(
            data_ws,
            min_col=1,
            min_row=2,
            max_row=1 + len(series)
        )
        chart.set_categories(categories)

        ws.add_chart(
            chart,
            "A1"
        )

    return wb


def get_files(path):
    """Return all files in a directory."""

    return [
        f
        for f in path.iterdir()
        if not f.is_dir()
    ]


def get_folders(path):
    """Return all subdirectories."""

    return [
        p
        for p in path.iterdir()
        if p.is_dir()
    ]


def add_percentage_to_existing_stats(
    output,
    runs_df
):
    """
    Open an existing Excel file and add/update the
    '% Within ±0.1 ms' column in the Statistics sheet.

    Does not rewrite the raw data or other sheets.
    """

    wb = load_workbook(output)

    if "Statistics" not in wb.sheetnames:

        print(
            f"[WARN] No Statistics sheet found "
            f"in {output}"
        )

        wb.close()
        return

    ws = wb["Statistics"]

    # Find the next empty column
    percentage_col = ws.max_column + 1

    # Add header
    header = ws.cell(
        row=1,
        column=percentage_col,
        value="% Within ±0.1 ms"
    )

    header.font = Font(
        name="Arial",
        bold=True,
        color="FFFFFF"
    )

    header.fill = PatternFill(
        "solid",
        fgColor="1F4E79"
    )

    header.alignment = Alignment(
        horizontal="center"
    )

    # Calculate percentage for each run
    for row, run_name in enumerate(
        runs_df.columns,
        2
    ):

        s = runs_df[
            run_name
        ].dropna()

        if len(s) == 0:

            percentage = 0

        else:

            mean = s.mean()

            within_range = (
                (s >= mean - 0.1)
                & (s <= mean + 0.1)
            )

            percentage = (
                within_range.mean()
                * 100
            )

        cell = ws.cell(
            row=row,
            column=percentage_col,
            value=round(
                percentage,
                2
            )
        )

        cell.font = Font(
            name="Arial"
        )

        cell.alignment = Alignment(
            horizontal="center"
        )

        # Match existing alternating row colors
        fill_color = (
            "D6E4F0"
            if row % 2 == 0
            else "FFFFFF"
        )

        cell.fill = PatternFill(
            "solid",
            fgColor=fill_color
        )

    # Set column width
    ws.column_dimensions[
        get_column_letter(
            percentage_col
        )
    ].width = 20

    wb.save(output)
    wb.close()

    print(
        f"[INFO] Updated Statistics sheet "
        f"in {output}"
    )


def iter_target_files(path):
    """Yield files to process for a given root directory."""

    path = Path(path)

    if path.is_file():
        return [path]

    files = sorted(
        get_files(path),
        key=lambda p: p.name
    )

    if files:
        return files

    collected = []

    for child in sorted(
        get_folders(path),
        key=lambda p: p.name
    ):
        collected.extend(
            iter_target_files(child)
        )

    return collected


def main(folder=None, edit=False):

    all_runs = []

    default_path = Path(
        "/home/robotics4farmers/Dev/Robot/Control_loop"
    )

    path = Path(folder) if folder is not None else default_path

    files = iter_target_files(path)

    if len(files) == 0:
        print(f"[WARN] No CSV files found in {path}")
        return

    for file in files:

        if edit:

            if "xlsx" in str(file):

                input_file = (
                    str(file)
                    .split("_results")[0]
                    + ".csv"
                )

                print(
                    input_file
                )

                runs = load_runs(
                    input_file,
                    0.01
                )

                all_runs.append(
                    runs
                )

                runs_df = build_dataframe(
                    runs
                )

                add_percentage_to_existing_stats(
                    file,
                    runs_df
                )

        else:
            if "xlsx" in str(file):
                continue

            if "100/" in str(file):
                period = 0.1
            else:
                period = 0.01

            sp = str(file).split(
                '.'
            )

            output = (
                sp[0]
                + "_results.xlsx"
            )

            print(output)

            print(
                f"[INFO] Loading runs "
                f"from {str(file)}"
            )

            runs = load_runs(
                file,
                period
            )

            all_runs.append(
                runs
            )

            runs_df = build_dataframe(
                runs
            )

            print(
                f"[INFO] Combined "
                f"{len(runs)} runs, "
                f"{len(runs_df)} max frames"
            )

            # -------------------------------------------------
            # Create workbook directly.
            #
            # Previously this used:
            #
            # runs_df.to_excel(...)
            #
            # That fails when runs_df has more than
            # 1,048,576 rows.
            #
            # We don't need pandas to write the initial sheet
            # because all sheets are created with openpyxl below.
            # -------------------------------------------------

            wb = load_workbook(
                output
            ) if os.path.exists(
                output
            ) else None

            wb = Workbook()

            # Remove the default worksheet
            default_ws = wb.active
            wb.remove(default_ws)

            # -------------------------------------------------
            # Add sheets in order:
            #
            # Statistics
            # Raw Latency (µs) 1
            # Raw Latency (µs) 2
            # ...
            # Histogram
            # Histogram Data
            # Execution_ms Chart
            # Execution_ms Data
            # Period_ms Chart
            # Period_ms Data
            # -------------------------------------------------

            add_stats_sheet(
                wb,
                runs_df,
                period * 100
            )

            add_chart_sheet(
                wb,
                runs_df
            )

            add_max_chart_sheet(
                wb,
                runs_df
            )

            wb.save(
                output
            )

            wb.close()

            print(
                f"[INFO] Saved {output}"
            )

            print(
                f"[INFO] Sheets: "
                f"Statistics, "
                f"Histogram, "
                f"Execution_ms Chart, "
                f"Period_ms Chart"
            )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description=(
            "Process latency CSV files from the default Control_loop "
            "directory or a specific folder."
        )
    )
    parser.add_argument(
        "folder",
        nargs="?",
        type=Path,
        default=None,
        help=(
            "Optional folder to process instead of the default "
            "Control_loop tree."
        )
    )
    parser.add_argument(
        "--edit",
        action="store_true",
        help=(
            "Update existing Excel statistics sheets instead of creating "
            "new result files."
        )
    )
    args = parser.parse_args()

    main(folder=args.folder, edit=args.edit)
