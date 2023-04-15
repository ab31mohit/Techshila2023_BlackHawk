import subprocess

# define array
my_array = [1, 2, 3, 4, 5]

# call other script and pass array as argument
subprocess.call(["python", "recieve.py", *map(str, my_array)])
