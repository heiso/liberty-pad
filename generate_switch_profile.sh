#!/bin/bash

# Script to generate switch profile lookup table
# Usage: ./generate_switch_profile.sh <idle_value> [max_value]
# idle_value: integer between 0 and 3300 (representing mV)
# max_value: integer between idle_value and 3300 (optional, defaults to 3300)

if [ $# -lt 1 ] || [ $# -gt 2 ]; then
    echo "Usage: $0 <idle_value> [max_value]"
    echo "idle_value must be between 0 and 3300"
    echo "max_value must be between idle_value and 3300 (optional, defaults to 3300)"
    exit 1
fi

idle_value=$1
max_value=${2:-3300}  # Default to 3300 if not provided

# Validate idle_value input
if ! [[ "$idle_value" =~ ^[0-9]+$ ]] || [ "$idle_value" -lt 0 ] || [ "$idle_value" -gt 3300 ]; then
    echo "Error: idle_value must be an integer between 0 and 3300"
    exit 1
fi

# Validate max_value input
if ! [[ "$max_value" =~ ^[0-9]+$ ]] || [ "$max_value" -lt "$idle_value" ] || [ "$max_value" -gt 3300 ]; then
    echo "Error: max_value must be an integer between $idle_value and 3300"
    exit 1
fi

# Calculate the range for mapping (from idle_value to max_value)
active_range=$((max_value - idle_value))

# Generate the C file header
cat > main/switch-profile.c << 'EOF'
#include <stdint.h>
#include <stdio.h>

// max value is 3300 for 3.3V
const uint8_t switch_profile[3301] = {
EOF

# Generate the array values
echo -n "  " >> main/switch-profile.c

for i in {0..3300}; do
    if [ $i -le $idle_value ]; then
        # Below or at idle value: output 0
        value=0
    elif [ $i -ge $max_value ]; then
        # At or above max value: output 255
        value=255
    else
        # Between idle_value and max_value: map linearly from 0 to 255
        # Formula: (current_position - idle_value) * 255 / active_range
        if [ $active_range -eq 0 ]; then
            value=255
        else
            value=$(( (i - idle_value) * 255 / active_range ))
        fi
    fi
    
    # Add comma except for last element
    if [ $i -eq 3300 ]; then
        echo -n "$value" >> main/switch-profile.c
    else
        echo -n "$value, " >> main/switch-profile.c
    fi
    
    # Add newline every 50 values for readability
    if [ $((i % 50)) -eq 49 ] && [ $i -ne 3300 ]; then
        echo "" >> main/switch-profile.c
        echo -n "  " >> main/switch-profile.c
    fi
done

# Close the array and file
cat >> main/switch-profile.c << 'EOF'

};
EOF

echo "Switch profile generated in main/switch-profile.c with idle value: $idle_value, max value: $max_value"
echo "Values 0-$idle_value: 0"
echo "Values $((idle_value + 1))-$((max_value - 1)): mapped 0-254"
echo "Values $max_value-3300: 255"