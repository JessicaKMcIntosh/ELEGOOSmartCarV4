import doctest

# Run tests to verify the Commands return the correct JSON.
print('Testing the Smart Car Commands...')
Results = doctest.testfile("test_Commands.txt")
print('Smart Car Commands:', Results)