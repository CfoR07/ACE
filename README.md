# Station-Based Autonomous Firefighting Agent (Classical AI Search)

An autonomous firefighting agent simulation implementing classical AI techniques to detect, prioritize, and extinguish building fires while navigating through obstacles using **A\* Search**.

## Features
- **A\* Pathfinding**: Optimal 4-directional navigation avoiding walls and active fire hazards.
- **Rule-Based Fire Priority Engine**: Evaluates fire clusters based on zone importance, cluster severity, and proximity.
- **Dynamic Fire Dynamics**: Simulates 4-directional fire propagation and staggered fire bursts.
- **Tactical Extinguishing**: Evaluates and deploys Single attacks vs. Splash attacks based on cost efficiency.
- **Inspection & Base Return**: Retains memory of extinguished zones to inspect for rekindling before returning safely to station.
- **Interactive Streamlit UI**: Dark-themed grid interface with real-time telemetry, memory inspection, and movement animation.

## Project Structure
```text
PROJECT/
├── functions.py      # Core AI search, fire dynamics, and simulation engine
├── app.py            # Streamlit dashboard & interactive grid visualization
├── requirements.txt  # Project dependencies
└── README.md         # Documentation
```

## Setup & Running
1. Install dependencies:
```bash
pip install -r requirements.txt
```

2. Run the application:
```bash
streamlit run app.py
```
