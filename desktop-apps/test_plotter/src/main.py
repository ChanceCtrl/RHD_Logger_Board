import argparse
import csv
import json

from mcap.records import Schema
from mcap.writer import Writer

NUM_CHANNELS = 64
NUM_REGS = 32

# Your timestamps increment by ~22-23.
# If these are microseconds, use 1000.
#
# If the timestamps are actually in another unit, change this.
TIMESTAMP_SCALE_NS = 1_000


def csv_to_mcap(input_csv, output_mcap):
    """
    Convert an RHD CSV into an MCAP that Foxglove can plot.

    Command == 0:
        Reg 0  -> RHD_A = channel 1,  RHD_B = channel 33
        Reg 1  -> RHD_A = channel 2,  RHD_B = channel 34
        ...
        Reg 31 -> RHD_A = channel 32, RHD_B = channel 64

    Each MCAP message contains all 64 channels as JSON fields.
    """

    # ------------------------------------------------------------------
    # Read CSV
    # ------------------------------------------------------------------

    samples = {}

    with open(input_csv, "r", newline="") as f:
        reader = csv.DictReader(f)

        required_columns = {
            "Timestamp",
            "Command",
            "Reg",
            "RHD_A",
            "RHD_B",
        }

        missing = required_columns - set(reader.fieldnames or [])

        if missing:
            raise ValueError(f"CSV is missing required columns: {sorted(missing)}")

        for line_number, row in enumerate(reader, start=2):
            try:
                timestamp = int(row["Timestamp"])
                command = int(row["Command"])
                reg = int(row["Reg"])
            except (TypeError, ValueError) as e:
                print(
                    f"Warning: skipping line {line_number}: "
                    f"invalid Timestamp/Command/Reg: {e}"
                )
                continue

            # Only Command 0 contains the 64-channel data.
            if command != 0:
                continue

            if not 0 <= reg < NUM_REGS:
                print(f"Warning: skipping line {line_number}: invalid Reg={reg}")
                continue

            # Create a sample for this timestamp if necessary.
            if timestamp not in samples:
                samples[timestamp] = {}

            # ----------------------------------------------------------
            # RHD_A -> channels 1-32
            # ----------------------------------------------------------

            rhd_a = row.get("RHD_A")

            if rhd_a is not None and rhd_a.strip() != "":
                try:
                    value = int(rhd_a)
                    value = max(0, min(65535, value))

                    channel = reg + 1
                    samples[timestamp][f"channel_{channel}"] = value

                except ValueError:
                    print(f"Warning: line {line_number}: invalid RHD_A={rhd_a!r}")

            # ----------------------------------------------------------
            # RHD_B -> channels 33-64
            # ----------------------------------------------------------

            rhd_b = row.get("RHD_B")

            if rhd_b is not None and rhd_b.strip() != "":
                try:
                    value = int(rhd_b)
                    value = max(0, min(65535, value))

                    channel = reg + 33
                    samples[timestamp][f"channel_{channel}"] = value

                except ValueError:
                    print(f"Warning: line {line_number}: invalid RHD_B={rhd_b!r}")

    if not samples:
        raise ValueError("No Command == 0 channel data found.")

    # ------------------------------------------------------------------
    # Normalize timestamps
    # ------------------------------------------------------------------

    timestamps = sorted(samples.keys())
    first_timestamp = timestamps[0]

    # ------------------------------------------------------------------
    # Create JSON schema
    # ------------------------------------------------------------------

    schema_fields = ",\n".join(
        f'    "channel_{channel}": {{"type": "number"}}'
        for channel in range(1, NUM_CHANNELS + 1)
    )

    schema = f"""
{{
  "type": "object",
  "properties": {{
{schema_fields}
  }}
}}
""".strip()

    # ------------------------------------------------------------------
    # Write MCAP
    # ------------------------------------------------------------------

    with open(output_mcap, "wb") as f:
        writer = Writer(f)

        writer.start(profile="rhd64", library="rhd-csv-to-mcap")

        # Register the JSON schema.
        schema_id = writer.register_schema(
            name="RHD64",
            encoding="jsonschema",
            data=schema.encode("utf-8"),
        )

        # Register a single channel containing the 64-channel sample.
        channel_id = writer.register_channel(
            topic="/rhd/channels",
            message_encoding="json",
            schema_id=schema_id,
            metadata={
                "description": "64-channel RHD ADC data",
                "channel_count": "64",
            },
        )

        # ------------------------------------------------------------------
        # Write samples
        # ------------------------------------------------------------------

        sequence = 0

        for timestamp in timestamps:
            # Convert device timestamp to MCAP nanoseconds.
            log_time = (timestamp - first_timestamp) * TIMESTAMP_SCALE_NS

            # Start with all 64 channels as null.
            #
            # Missing fields are left as null rather than carrying
            # stale data from the previous sample.
            message = {
                f"channel_{channel}": None for channel in range(1, NUM_CHANNELS + 1)
            }

            # Insert values that were actually present in this sample.
            message.update(samples[timestamp])

            payload = json.dumps(message, separators=(",", ":")).encode("utf-8")

            writer.add_message(
                channel_id=channel_id,
                log_time=log_time,
                publish_time=log_time,
                sequence=sequence,
                data=payload,
            )

            sequence += 1

        writer.finish()

    print()
    print(f"Wrote: {output_mcap}")
    print(f"Samples: {len(timestamps)}")
    print(f"Channels: {NUM_CHANNELS}")
    print(f"First CSV timestamp: {first_timestamp}")
    print()
    print("Foxglove topic:")
    print("  /rhd/channels")
    print()
    print("Plot fields:")
    print("  channel_1 ... channel_64")


def main():
    parser = argparse.ArgumentParser(
        description=(
            "Convert a 64-channel RHD CSV into an MCAP file suitable for Foxglove."
        )
    )

    parser.add_argument("input_csv", help="Input RHD CSV")

    parser.add_argument(
        "-o",
        "--output",
        default="rhd.mcap",
        help="Output MCAP filename (default: rhd.mcap)",
    )

    args = parser.parse_args()

    csv_to_mcap(args.input_csv, args.output)


if __name__ == "__main__":
    main()
