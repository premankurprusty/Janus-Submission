Task 1 
main.py: it imports the Xbee libraries and reads the incoming data.

It pushes the raw string into telemetry.py where it deconstructs the string.

That list of values is then given to plotter.py which animates the graph using matplotlib
