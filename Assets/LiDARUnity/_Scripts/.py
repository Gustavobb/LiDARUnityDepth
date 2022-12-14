class Vector3:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
    
    def div(self, other):
        return Vector3(self.x / other, self.y / other, self.z / other)

    def __mul__(self, other):
        return Vector3(self.x * other, self.y * other, self.z * other)
    
    def __truediv__(self, other):
        return Vector3(self.x / other, self.y / other, self.z / other)
    
    def __add__(self, other):
        return Vector3(self.x + other.x, self.y + other.y, self.z + other.z)
    
    def __sub__(self, other):
        return Vector3(self.x - other.x, self.y - other.y, self.z - other.z)
    
    def round(self):
        return Vector3(round(self.x), round(self.y), round(self.z))
    
    def integer(self):
        return Vector3(int(self.x), int(self.y), int(self.z))

def numberToBase(n, b):
    if n == 0:
        return [0]
    digits = []
    while n:
        digits.append(int(n % b))
        n //= b
    return digits[::-1]

def find_chunk_pos(vertice, chunk_size, divide_chunk_into):
    print("Per axis (x, y, z) " + str(chunk_size / divide_chunk_into))
    chunk_pos = vertice.div(chunk_size).round()
    print("Chunk pos:", chunk_pos.x, chunk_pos.y, chunk_pos.z)
    return chunk_pos

def find_idx_by_subchunk(chunk_pos, vertice, chunk_size, divide_chunk_into):
    vertices_in_sub_chunk = Vector3((vertice.x / chunk_size - chunk_pos.x + 1 / 2),
                                    (vertice.y / chunk_size - chunk_pos.y + 1 / 2),
                                    (vertice.z / chunk_size - chunk_pos.z + 1 / 2))
    vertices_in_sub_chunk *= divide_chunk_into

    pos_in_array = int(vertices_in_sub_chunk.x) + int(vertices_in_sub_chunk.y) * divide_chunk_into + int(vertices_in_sub_chunk.z) * divide_chunk_into * divide_chunk_into
    return pos_in_array if pos_in_array < divide_chunk_into ** 3 else -1

def find_mean_subchunk_pos_by_id(chunk_pos, chunk_size, divide_chunk_into, idx):
    x = idx % divide_chunk_into
    z = idx // divide_chunk_into // divide_chunk_into
    y = idx // divide_chunk_into % divide_chunk_into
    print("x, y, z:", x, y, z)

    half_chunk_size = Vector3(chunk_size, chunk_size, chunk_size) / 2
    world_chunk_pos = (chunk_pos * chunk_size) - half_chunk_size
    print("World chunk pos:", world_chunk_pos.x, world_chunk_pos.y, world_chunk_pos.z)
    print("Half chunk size:", half_chunk_size.x, half_chunk_size.y, half_chunk_size.z)
    return world_chunk_pos + Vector3(x, y, z) * chunk_size / divide_chunk_into + half_chunk_size / divide_chunk_into

vertice = Vector3(.4, .4, .4)
chunk_size = 1
divide_chunk_into = 3

print("Vertice:", vertice.x, vertice.y, vertice.z)
chunk_pos = find_chunk_pos(vertice, chunk_size, divide_chunk_into)
idx = find_idx_by_subchunk(chunk_pos, vertice, chunk_size, divide_chunk_into)
print("Idx:", idx)
result = find_mean_subchunk_pos_by_id(chunk_pos, chunk_size, divide_chunk_into, idx)
print(result.x, result.y, result.z)