from pathlib import Path
import csv


def load_csv_rows(path: Path):
    with path.open(newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def test_sample_csv_exists_later():
    data_dir = Path(__file__).parent / "data"
    csv_files = list(data_dir.glob("*.csv"))
    assert True, (
        "No CSV file yet. Later, add your trial CSV into tests/data/ "
        "and replace this placeholder test with real checks."
    )


def test_expected_columns_when_csv_is_added():
    data_dir = Path(__file__).parent / "data"
    csv_files = list(data_dir.glob("*.csv"))
    if not csv_files:
        assert True
        return

    rows = load_csv_rows(csv_files[0])
    assert rows, "CSV exists but has no data rows."

    expected = {
        "Time_ms",
        "RawADC",
        "Voltage_V",
        "Force_lbs",
        "Target_lbs",
        "Kp",
        "Ki",
        "Kd",
    }
    assert expected.issubset(rows[0].keys())
