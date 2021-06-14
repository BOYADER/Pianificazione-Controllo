# Pianificazione e Controllo Waypoints: Guida al codice
Pianificazione del moto dell'AUV e implementazione di 5 controllori PI per il suo controllo.
La missione consiste nel raggiungere dei waypoints di cui le coordinate (Lat. Long. e Prof.) sono espresse nel file `mission.yaml`.

## Contenuti:
* [1. Requisiti](#1-requisiti)
* [2. Simulazione finale](#2-simulazione finale)
* [3. Simulazione intermedia](#3-simulazione intermedia)

## 1) Requisiti
Per eseguire correttamente la simulazione ed ottenere i risultati mostrati nel report, è opportuno assicurarsi di:

- Avere scaricato tutti e 3 i pkgs del Team Waypoints. Altrimenti eseguire:
     ```
        ~/catkin_ws/src:
        git clone https://github.com/BOYADER/Navigazione
        git clone https://github.com/BOYADER/Pianificazione-Controllo
        git clone https://github.com/BOYADER/Modellazione 
     ```
- Rendere eseguibili i nodi python del pkg pc_wp:
     ```
        ~/catkin_ws/src/Pianificazione-Controllo/pc_wp/scripts ~$ chmod +x *
     ```  
- Ritornare nel catkin workspace e lanciare il comando catkin_make
     ```
	~/catkin_ws/catkin_make
     ```
- Scaricare pymap3d e termcolor:
     ```
	python3 -m pip install pymap3d --- source: https://pypi.org/project/pymap3d/
	pip install termcolor ------------ source: https://pypi.org/project/termcolor/
     ```
## 2) Simulazione finale
Per lanciare la simulazione relativa al funzionamento dell'intero anello in ciclo chiuso , digitare da terminale:
 ```
      ~$ roslaunch pc_wp launch.launch
 ```
Il launch appartiene al pkg pc_wp e lancia i nodi principali dei pkgs del Team. Per visualizzare meglio la simulazione, vengono inoltre lanciati dei nodi rqt_plot. 
(https://github.com/BOYADER/Pianificazione-Controllo/blob/main/pc_wp/launch/launch.launch)
E' possibile plottare:

	- Stato vero e stato stimato.
	- Forze e Momenti generate dal blocco di Controllo.
	- Errore sulle singole grandezze.
	- Errore quadratico Medio (MSE) calcolato iterativamente al variare del tempo.

![alt text](https://github.com/BOYADER/Navigazione/blob/main/docs/rqt_graph.PNG)

## 3) Simulazione intermedia
Per lanciare la simulazione relativa all'integrazione tra Pianificazione-Controllo e Modellazione, digitare da terminale:
 ```
      ~$ roslaunch pc_wp test.launch
 ```
(https://github.com/BOYADER/Pianificazione-Controllo/blob/main/pc_wp/launch/test.launch)

![alt text](https://github.com/BOYADER/Pianificazione-Controllo/blob/main/testlaunch.png)

Per spiegazioni dettagliate circa il codice e i risultati, si rimanda al report del modulo Pianificazione-Controllo presente nella stessa cartella di README.md
