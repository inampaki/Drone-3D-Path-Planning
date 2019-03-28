import csv
import matplotlib.pyplot as plt
def get_long_lat():
    lat = None
    lon = None
    with open('./colliders.csv', newline='\n') as file_descripter:
        reader = csv.reader(file_descripter)
        first_row = next(reader)
    file_descripter.close()
    for cell in first_row:
        try:
            if cell.index("lat0") != -1:
                lat = float((cell.strip()).split(" ")[1])     
        except:
            try:
                if cell.index("lon0") != -1:
                    lon = float((cell.strip()).split(" ")[1])
            except:
                print('Nor a Lonitude nor a latitude')                  
    return(lat, lon)

def plot_route(path, pruned_path, start_ne, goal_ne):

    plt.plot(start_ne[1], start_ne[0], 'gx', markersize=15)
    plt.plot(goal_ne[1], goal_ne[0], 'gx', markersize=15)

    plt.xlabel('EAST', fontsize=20)
    plt.ylabel('NORTH', fontsize=20)