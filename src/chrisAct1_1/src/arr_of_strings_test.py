l_of_strings = []

def array2string(array):
	if len(array) >= 1:	
		result = "\'  ,  \'".join(array)
		result = "[\'" + result
		result += "\']"
		return result
	else:
		return "[]"

def str2array(str):
	return eval(str)

print("testing string conversion")
print("original array is :")
print(l_of_strings)
to_str = array2_string(l_of_strings)
print("converted to string is:") 
print(to_str)
to_arr = eval(to_str)
print("converted to array again is:") 
print(to_arr)
