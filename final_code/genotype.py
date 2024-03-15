import random
import numpy as np
import copy

class Genotype:
    def __init__(self, numSegments, segment_dimensions, moves, protocol, gear, friction):
        if not isinstance(numSegments, int):
            raise ValueError("numSegments must be an integer")
        if len(segment_dimensions) != numSegments:
            raise ValueError("segment_dimensions list must be the same length as numSegments")
        for dim in segment_dimensions:
            if not (isinstance(dim, tuple) and len(dim) == 3):
                raise ValueError("Each segment dimension must be a 3-tuple")
                
        self.numSegments = numSegments
        self.segment_dimensions = segment_dimensions
        self.moves = moves
        self.protocol = protocol
        self.gear = gear
        self.friction = friction

    @classmethod
    def randomize(cls):
        numSegments = random.randint(3, 15)  # Randomly choose an integer between 2 and 8
        segment_dimensions = [(random.uniform(0.01, 0.05), random.uniform(0.01, 0.05), random.uniform(0.01, 0.05)) for _ in range(numSegments)]
        moves = np.array([random.uniform(0.25, 1.00) for _ in range(numSegments-1)]) 
        protocol = 1
        gear = random.randint(30, 60)
        friction = [(random.uniform(0.5, 2), random.uniform(0, 0.01), random.uniform(0, 0.005)) for _ in range(numSegments)]
        return cls(numSegments, segment_dimensions, moves, protocol, gear, friction)
    
    @classmethod
    def from_dict(cls, genotype_dict):
        #Convert moves back to a numpy array (annoying)
        moves = np.array(genotype_dict['moves'])
        
        #Convert each segment dimension from list to tuple (Why did i do it like this)
        segment_dimensions = [tuple(dim) for dim in genotype_dict['segment_dimensions']]
        
        return cls(numSegments=genotype_dict['numSegments'],
                segment_dimensions=segment_dimensions,
                moves=moves,
                protocol=genotype_dict['protocol'],
                gear=genotype_dict['gear'],
                friction = genotype_dict['friction'])
        
    def to_dict(self):
        """Converts the Genotype instance into a dictionary."""
        return {
            "numSegments": self.numSegments,
            "segment_dimensions": [list(dim) for dim in self.segment_dimensions], 
            "moves": self.moves.tolist(), 
            "protocol": self.protocol,
            "gear": self.gear,
            "friction": [list(f) for f in self.friction] 
        }

        
    def clone(self):
        """Creates a deep copy of this genotype, including cloning numpy arrays properly."""
        cloned_segment_dimensions = copy.deepcopy(self.segment_dimensions)
        cloned_moves = np.copy(self.moves)
        cloned_friction = copy.deepcopy(self.friction)
        return Genotype(self.numSegments, cloned_segment_dimensions, cloned_moves, self.protocol, self.gear, cloned_friction)
    
    def mutate(self, mutationRate):
        #Clone the current
        mutated_clone = self.clone()

        #mutate to the clone
        lower_bound, upper_bound = 1 - 0.1 * mutationRate, 1 + 0.1 * mutationRate
        
        mutated_clone.segment_dimensions = [(max(0.01, dim[0] * random.uniform(lower_bound, upper_bound)),
                                             max(0.01, dim[1] * random.uniform(lower_bound, upper_bound)),
                                             max(0.01, dim[2] * random.uniform(lower_bound, upper_bound)))
                                            for dim in mutated_clone.segment_dimensions]
        
        mutated_clone.moves = np.clip(mutated_clone.moves * np.random.uniform(lower_bound, upper_bound, size=mutated_clone.moves.shape), 0.25, 1.00)
        
        gear_mutation_range = 5 + mutationRate
        mutated_clone.gear = max(30, min(60, mutated_clone.gear + random.randint(-gear_mutation_range, gear_mutation_range)))
        
        mutated_clone.friction = [(f[0] * random.uniform(lower_bound, upper_bound),
                                   f[1] * random.uniform(lower_bound, upper_bound),
                                   f[2] * random.uniform(lower_bound, upper_bound))
                                  for f in mutated_clone.friction]
        
        return mutated_clone

    def __str__(self):
        return f"Genotype(numSegments={self.numSegments},\n segment_dimensions={self.segment_dimensions},\n  moves={self.moves.tolist()},\n  protocol={self.protocol},\n  gear={self.gear},\n friction={self.friction}\n )"
