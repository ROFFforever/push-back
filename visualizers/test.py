def get_axis_offset(angle):
    # (angle + 45) % 90 shifts the range to 0-89
    # Subtracting 45 centers it to -45 to 44
    return (angle + 45) % 90 - 45

print(get_axis_offset(85.5))