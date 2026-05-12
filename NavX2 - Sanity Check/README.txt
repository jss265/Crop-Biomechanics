The purpose of this demo is to provide a quick sanity check for the NavX2. The NavX2 will send packets as strings like this: 

$X-0.0125Y-0.0537Z0.2715w-32.7420x-0.0420y-0.3200z1.2340T606477. 

Here are a brief description of the python files:
- Serial_Reader.py: Prints the serial stream from NavX2

- Serial_Logger.py: Records CSV files of motions from NavX2
Recording begins immediately and stops on ESC

- CSV_Reader.py: displays a matplot lib path of all timesteps in a CSV
Will print all data files, copy and paste the desired file
Description, time recorded, and duration also displayed

- Live_Reader.py: Live visualization of the path and orientation of the NavX2 
I threw this one into ChatGPT and haven’t figured out how it works yet
Hit reset button on the NavX2 to reset to origin
May need to rotate camera view with right click and drag
