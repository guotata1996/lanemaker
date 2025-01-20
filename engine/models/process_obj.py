model = 'military'

x_min = y_min = z_min = float('inf')
x_max = y_max = z_max = -float('inf')

with open(model + '_original.obj', 'r') as in_obj:
    for line in in_obj:
        if line.startswith('v '):
            _, x, y, z = line.split(' ')
            x = float(x)
            y = float(y)
            z = float(z)
            x_min = min(x, x_min)
            y_min = min(y, y_min)
            z_min = min(z, z_min)
            x_max = max(x, x_max)
            y_max = max(y, y_max)
            z_max = max(z, z_max)

original_length = z_max - z_min
x_offset = -(x_max + x_min) / 2
y_offset = -y_min
z_offset = -(z_max + z_min) / 2
scale = 4.6 / original_length

with open(model + '_original.obj', 'r') as in_obj:
    with open(model + '.obj', 'w') as out_obj:
        for line in in_obj:
            if line.startswith('v '):
                _, x, y, z = line.split(' ')
                x = (float(x) + x_offset) * scale
                y = (float(y) + y_offset) * scale
                z = (float(z) + z_offset) * scale
                out_obj.write(f'v {x} {y} {z}\n')
            else:
                out_obj.write(line)
