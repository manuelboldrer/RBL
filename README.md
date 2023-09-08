# Overview


<img src="videos/tud.gif" width="300" height="200"/><img src="videos/random.gif" width="300" height="200"/><img src="videos/circle.gif" width="300" height="200"/>


This is the implementation of the Heuristic Lloyd-based algorithm (HLB) for multi-robot navigation. 
Multi-Robot Navigation


The simulator can run on all major platforms (Windows, Linux, and macOS). All that is required is cloning the repository and installing some necessary dependencies.


    git clone https://github.com/manuelboldrer/HLB

### Reference
This repository is the source code of the paper: 

"Distributed Networkless Multi-Robot Navigation in Crowded Cooperative Environments."
Manuel Boldrer, Alvaro Serra-Gomez, Lorenzo Lyons, Javier Alonso-Mora, Laura Ferranti. Under Review at ...

**[`PDF_arxiv`](https://arxiv.org/pdf/????)** 

**[`Youtube video`](https://youtube/????)** 
 

### Prerequisites
- Python >3.9 & Pip
- Python Packages as defined in [requirements.txt](requirements.txt) 

## Setup
Install Python Packages
    
    pip install -r requirements.txt

Test Simulation with render

    python3 main.py -render

Test Simulation writing .txt file

    python3 main.py -writefile

## Acknowledgements

Giovanni Franzese,
Alvaro Serra Gomez,
Lorenzo Lyons,
Laura Ferranti,
Javier Alonso-Mora.







