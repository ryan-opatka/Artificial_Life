import random

class Genotype:
    def __init__(self, numSegments, segment_dimensions):
        if not isinstance(numSegments, int):
            raise ValueError("numSegments must be an integer")
        if len(segment_dimensions) != numSegments:
            raise ValueError("segment_dimensions list must be the same length as numSegments")
        for dim in segment_dimensions:
            if not (isinstance(dim, tuple) and len(dim) == 3):
                raise ValueError("Each segment dimension must be a 3-tuple")
                
        self.numSegments = numSegments
        self.segment_dimensions = segment_dimensions

    @classmethod
    def randomize(cls):
        numSegments = random.randint(3, 4)  # Randomly choose an integer between 2 and 8
        segment_dimensions = [(random.uniform(0.01, 0.1), random.uniform(0.01, 0.1), random.uniform(0.01, 0.1)) for _ in range(numSegments)]
        return cls(numSegments, segment_dimensions)

    def __str__(self):
        return f"Genotype(numSegments={self.numSegments}, segment_dimensions={self.segment_dimensions})"

# Create a randomized genotype and print it