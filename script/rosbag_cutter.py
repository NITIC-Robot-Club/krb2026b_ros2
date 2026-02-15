#!/usr/bin/env python3

import sys
from rosbag2_py import SequentialReader, SequentialWriter
from rosbag2_py import StorageOptions, ConverterOptions

def open_reader(uri):
    storage_options = StorageOptions(
        uri=uri,
        storage_id='mcap'
    )

    converter_options = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )

    reader = SequentialReader()
    reader.open(storage_options, converter_options)
    return reader, storage_options, converter_options


def main():
    if len(sys.argv) != 2:
        print("Usage: python3 rosbag_cutter.py <input_bag_dir>")
        sys.exit(1)

    input_uri = sys.argv[1]

    reader, storage_options, converter_options = open_reader(input_uri)

    topics = reader.get_all_topics_and_types()

    print("\n=== Topics ===")
    for t in topics:
        print(f"{t.name} : {t.type}")

    # --- first pass: get time range ---
    if not reader.has_next():
        print("Bag is empty")
        sys.exit(1)

    topic, data, first_t = reader.read_next()
    last_t = first_t

    while reader.has_next():
        _, _, last_t = reader.read_next()

    total_duration = (last_t - first_t) / 1e9

    print("\n=== Time Info ===")
    print(f"Start time (ns): {first_t}")
    print(f"End time   (ns): {last_t}")
    print(f"Total duration (s): {total_duration:.3f}")

    start_offset = float(input("\nStart offset (sec): "))
    duration = float(input("Duration (sec): "))
    output_uri = input("Output bag name: ")

    start_ns = first_t + int(start_offset * 1e9)
    end_ns = start_ns + int(duration * 1e9)

    # --- reopen reader (resetの代わり) ---
    reader, _, _ = open_reader(input_uri)

    writer = SequentialWriter()
    writer.open(
        StorageOptions(uri=output_uri, storage_id='mcap'),
        converter_options
    )

    for t in topics:
        writer.create_topic(t)

    print("\nCutting...")

    while reader.has_next():
        topic, data, timestamp = reader.read_next()

        if timestamp < start_ns:
            continue
        if timestamp > end_ns:
            break

        writer.write(topic, data, timestamp)

    print("Done.")


if __name__ == "__main__":
    main()
