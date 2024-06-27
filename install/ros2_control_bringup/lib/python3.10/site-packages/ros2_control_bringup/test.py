import random

# word1 = random.uniform(-1,1)
# word2 = random.uniform(-1,1)
# word3 = random.uniform(-1,1)

# #example using f strings
# print(f"{word1} {word2} {word3}")
# print("aa"+"aa")
print(random.uniform(*random.choice([(1.0, 1.5), (9.0, 9.5), (21.0, 21.5)])))