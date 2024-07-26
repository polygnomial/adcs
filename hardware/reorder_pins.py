def reorder_pins(input_file, output_file):
    with open(input_file, 'r') as file:
        lines = file.readlines()

    # Parse the lines
    lines = [line.strip().split() for line in lines]

    # Separate by connector
    connectors = {}
    for line in lines:
        connector, pin, pin_label, signal = line
        if connector not in connectors:
            connectors[connector] = []
        connectors[connector].append((int(pin), pin_label, signal))

    # Reorder the pins
    reordered_lines = []
    for connector, pins in connectors.items():
        # Separate odd and even pins
        odd_pins = [pin for pin in pins if pin[0] % 2 == 1]
        even_pins = [pin for pin in pins if pin[0] % 2 == 0]

        # Sort pins
        odd_pins.sort()
        even_pins.sort()

        # Combine back
        ordered_pins = odd_pins + even_pins
        for pin in ordered_pins:
            reordered_lines.append(f"{connector}\t{pin[0]}\t{pin[1]}\t{pin[2]}\n")

    # Write the reordered lines to the output file
    with open(output_file, 'w') as file:
        for line in reordered_lines:
            file.write(line + '\n')

# Usage
input_file = 'pinout.txt'
output_file = 'pinout_output.txt'
reorder_pins(input_file, output_file)
