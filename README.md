# 3D Visualization Project

## Overview
The 3D Visualization Project is designed to visualize sensor data received via UDP. It processes incoming data from a mobile device and renders a 3D representation of the sensor's orientation in real-time. This project utilizes Python and various libraries to achieve smooth and interactive visualizations.

## Project Structure
```
3d-visualization-project
├── src
│   ├── main.py               # Main entry point of the application
│   ├── udp_reciver.py        # Handles receiving UDP packets
│   ├── sensor_fussion.py      # Processes sensor data and updates orientation
│   ├── visualisation.py       # Renders the visual representation of sensor data
│   └── utils
│       └── __init__.py       # Initializes the utils package
├── requirements.txt           # Lists project dependencies
├── README.md                  # Project documentation
└── .gitignore                 # Specifies files to ignore by Git
```

## Installation
To set up the project, follow these steps:

1. Clone the repository:
   ```
   git clone https://github.com/yourusername/3d-visualization-project.git
   ```
2. Navigate to the project directory:
   ```
   cd 3d-visualization-project
   ```
3. Install the required dependencies:
   ```
   pip install -r requirements.txt
   ```

## Usage
To run the application, execute the following command:
```
python src/main.py
```

Make sure your mobile device is configured to send UDP packets to the specified port (5006 in this case).

## Contributing
Contributions are welcome! Please feel free to submit a pull request or open an issue for any suggestions or improvements.

## License
This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.