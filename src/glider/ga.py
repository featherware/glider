# import pygad
# import numpy

# from vehicle import Vehicle

# """
# Using the points of the glider mesh as genes, define a GA statement that maximizes glider forward movement.
# """

# def fitness_func(ga_instance, solution, solution_idx):
#     # Calculating the fitness value of each solution in the current population.
#     # The fitness function calulates the sum of products between each input and its corresponding weight.
#     v = Vehicle(vertices=solution)
#     return v.fitness()

# fitness_function = fitness_func

# num_generations = 100 # Number of generations.
# num_parents_mating = 7 # Number of solutions to be selected as parents in the mating pool.

# # To prepare the initial population, there are 2 ways:
# # 1) Prepare it yourself and pass it to the initial_population parameter. This way is useful when the user wants to start the genetic algorithm with a custom initial population.
# # 2) Assign valid integer values to the sol_per_pop and num_genes parameters. If the initial_population parameter exists, then the sol_per_pop and num_genes parameters are useless.
# sol_per_pop = 50 # Number of solutions in the population.
# num_genes = len(function_inputs)

# last_fitness = 0
# def callback_generation(ga_instance):
#     global last_fitness
#     print("Generation = {generation}".format(generation=ga_instance.generations_completed))
#     print("Fitness    = {fitness}".format(fitness=ga_instance.best_solution()[1]))
#     print("Change     = {change}".format(change=ga_instance.best_solution()[1] - last_fitness))
#     last_fitness = ga_instance.best_solution()[1]

# # Creating an instance of the GA class inside the ga module. Some parameters are initialized within the constructor.
# ga_instance = pygad.GA(num_generations=num_generations,
#                        num_parents_mating=num_parents_mating, 
#                        fitness_func=fitness_function,
#                        sol_per_pop=sol_per_pop, 
#                        num_genes=num_genes,
#                        on_generation=callback_generation)

# # Running the GA to optimize the parameters of the function.
# ga_instance.run()

# # After the generations complete, some plots are showed that summarize the how the outputs/fitenss values evolve over generations.
# ga_instance.plot_fitness()

# # Returning the details of the best solution.
# solution, solution_fitness, solution_idx = ga_instance.best_solution()
# print("Parameters of the best solution : {solution}".format(solution=solution))
# print("Fitness value of the best solution = {solution_fitness}".format(solution_fitness=solution_fitness))
# print("Index of the best solution : {solution_idx}".format(solution_idx=solution_idx))

# prediction = numpy.sum(numpy.array(function_inputs)*solution)
# print("Predicted output based on the best solution : {prediction}".format(prediction=prediction))

# if ga_instance.best_solution_generation != -1:
#     print("Best fitness value reached after {best_solution_generation} generations.".format(best_solution_generation=ga_instance.best_solution_generation))

# # Saving the GA instance.
# filename = 'genetic' # The filename to which the instance is saved. The name is without extension.
# ga_instance.save(filename=filename)

# # Loading the saved GA instance.
# loaded_ga_instance = pygad.load(filename=filename)
# loaded_ga_instance.plot_fitness()