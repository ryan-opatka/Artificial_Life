
import subprocess

# rand = g.Genotype.randomize()

# print(rand)

# rand.mutate(3)

# print(rand)


def run(name, times):
    for _ in range(times):
        subprocess.run(['python', name])


run('gen1.py', 25)

run('evolve.py', 1)


