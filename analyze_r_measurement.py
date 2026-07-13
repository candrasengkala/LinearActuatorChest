"""
analyze_r_measurement.py

Analyzes CSV logs captured from LinAct::covarianceHelper() to estimate
the measurement noise covariance R for the Kalman filter.

Expected CSV format (from Serial monitor capture):
    raw_adc,position_mm
    512,123.4500
    512,123.4500
    513,123.5012
    ...

Usage:
    python analyze_r_measurement.py r_measurement.csv
    python analyze_r_measurement.py r_measurement.csv --lsb-mm 0.0488

If --lsb-mm is provided, the script also reports the theoretical
quantization noise floor (LSB^2 / 12) for comparison, useful when the
measured variance comes back at or near zero (ADC bit-locked case).
"""

import sys
import argparse
import numpy as np


def load_csv(path, debug=False):
    """
    Loads the CSV, skipping the PlatformIO monitor banner lines and
    the header row. Keeps only lines that parse as 'number,number'.
    Tolerant of CRLF, stray whitespace, quotes, and non-UTF8 bytes
    that sometimes sneak in from serial monitor captures.
    """
    raw_adc = []
    pos_mm = []
    rejected_samples = []

    # Sniff encoding from BOM. PowerShell's Tee-Object/Out-File default to
    # UTF-16LE with a BOM, which looks like 'ÿþ' + null-padded chars if
    # opened as plain UTF-8/latin-1 text.
    with open(path, "rb") as f:
        raw_bytes = f.read(4)

    if raw_bytes.startswith(b"\xff\xfe"):
        encoding = "utf-16"
    elif raw_bytes.startswith(b"\xfe\xff"):
        encoding = "utf-16-be"
    elif raw_bytes.startswith(b"\xef\xbb\xbf"):
        encoding = "utf-8-sig"
    else:
        encoding = "utf-8"

    try:
        with open(path, "r", encoding=encoding) as f:
            lines = f.readlines()
    except UnicodeDecodeError:
        with open(path, "r", encoding="latin-1") as f:
            lines = f.readlines()

    for line in lines:
        # Strip whitespace, stray quotes, and any CR left over.
        line = line.strip().strip('"').strip("'")
        if not line:
            continue
        # Split on comma, tolerate extra spaces around values.
        parts = [p.strip() for p in line.split(",")]
        if len(parts) != 2:
            if debug and len(rejected_samples) < 15:
                rejected_samples.append(line)
            continue
        try:
            adc_val = float(parts[0])  # float() also accepts plain ints
            pos_val = float(parts[1])
        except ValueError:
            # Skips header row ("raw_adc,position_mm") and any
            # banner/garbage lines from the serial monitor
            # (e.g. "--- Miniterm on ... ---").
            if debug and len(rejected_samples) < 15:
                rejected_samples.append(line)
            continue
        raw_adc.append(adc_val)
        pos_mm.append(pos_val)

    if not raw_adc:
        msg = (
            "No valid numeric rows found. Check the CSV format / "
            "that the header/banner lines are being skipped correctly."
        )
        if rejected_samples:
            msg += "\n\nFirst rejected lines (for debugging):\n"
            msg += "\n".join(f"  {i+1}: {repr(s)}" for i, s in enumerate(rejected_samples))
        else:
            msg += "\n\nThe file appears to be empty or unreadable."
        raise ValueError(msg)

    if debug and rejected_samples:
        print(f"[debug] Skipped {len(rejected_samples)}+ non-numeric line(s), e.g.:")
        for s in rejected_samples[:5]:
            print(f"  {repr(s)}")
        print()

    return np.array(raw_adc), np.array(pos_mm)


def analyze(raw_adc, pos_mm, lsb_mm=None):
    n = len(raw_adc)

    adc_mean = np.mean(raw_adc)
    adc_var = np.var(raw_adc, ddof=1)  # sample variance (N-1)
    adc_std = np.sqrt(adc_var)

    pos_mean = np.mean(pos_mm)
    pos_var = np.var(pos_mm, ddof=1)
    pos_std = np.sqrt(pos_var)

    unique_codes = np.unique(raw_adc)

    print("=" * 55)
    print(f"Samples analyzed: {n}")
    print("=" * 55)

    print("\n--- Raw ADC counts ---")
    print(f"Mean:               {adc_mean:.4f}")
    print(f"Variance:            {adc_var:.6f}  (counts^2)")
    print(f"Std dev:             {adc_std:.6f}  (counts)")
    print(f"Unique codes seen:   {len(unique_codes)}  {unique_codes.tolist() if len(unique_codes) <= 10 else '(>10 unique values)'}")

    print("\n--- Position (mm) ---")
    print(f"Mean:                {pos_mean:.6f} mm")
    print(f"Variance (R):        {pos_var:.8f} mm^2")
    print(f"Std dev:             {pos_std:.6f} mm")

    print("\n--- Interpretation ---")
    if len(unique_codes) == 1:
        print("WARNING: ADC output is bit-locked (single value across all")
        print("samples). Measured variance is 0 and NOT usable as R directly.")
        if lsb_mm is not None:
            quant_R = (lsb_mm ** 2) / 12.0
            print(f"\nUse the quantization noise floor instead:")
            print(f"  LSB size:          {lsb_mm} mm")
            print(f"  Quantization R:    {quant_R:.8f} mm^2  (LSB^2 / 12)")
        else:
            print("Re-run with --lsb-mm <value> to get a quantization-based")
            print("R estimate (LSB size = mechanical travel / 1024 for 10-bit ADC).")
    elif len(unique_codes) == 2:
        print("ADC is dithering between two adjacent codes only.")
        print("Measured variance is real but may be small; consider more")
        print("samples for a stable estimate.")
        print(f"\nSuggested R for Kalman filter: {pos_var:.8f} mm^2")
    else:
        print(f"Suggested R for Kalman filter: {pos_var:.8f} mm^2")

    if lsb_mm is not None and len(unique_codes) > 1:
        quant_R = (lsb_mm ** 2) / 12.0
        print(f"\n(For reference, theoretical quantization floor R = {quant_R:.8f} mm^2)")

    print("=" * 55)

    return {
        "n": n,
        "adc_mean": adc_mean,
        "adc_var": adc_var,
        "pos_mean": pos_mean,
        "pos_var": pos_var,
        "unique_codes": unique_codes,
    }


def main():
    parser = argparse.ArgumentParser(description="Analyze R-measurement CSV log.")
    parser.add_argument("csv_path", help="Path to the captured CSV log file")
    parser.add_argument(
        "--lsb-mm",
        type=float,
        default=None,
        help="ADC LSB size in mm (mechanical travel / 1024 for 10-bit ADC). "
             "Used to report a quantization noise floor if the ADC is bit-locked.",
    )
    parser.add_argument(
        "--debug",
        action="store_true",
        help="Print lines that failed to parse, to help diagnose CSV format issues.",
    )
    args = parser.parse_args()

    raw_adc, pos_mm = load_csv(args.csv_path, debug=args.debug)
    analyze(raw_adc, pos_mm, lsb_mm=args.lsb_mm)


if __name__ == "__main__":
    main()