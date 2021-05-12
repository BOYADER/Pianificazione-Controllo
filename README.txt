# Pianificazione-Controllo
Per eseguire e modificare il codice :

SE E' LA PRIMA VOLTA CHE TI CONNETTI AL QUESTO SERVER REMOTO

1. scegliere la directory in cui clonare la repository 'Pianificazione-Controllo' (es. ~/catkin_ws/src)
2. da terminale digitare
	git clone https://github.com/Boia-Dehr/Pianificazione-Controllo
(3. A regola ora dovreste lanciare il comando catkin_make a partire da ~/catkin_ws, cosi si costruisce il pacchetto penso.. che esiste già di fatto, ma non ancora nel ws)
4. Lavorate pure in locale sui vari file
5. dopo aver fatto le vostre modifiche, se volete mandarle in remoto, da terminale digitate
	git add <nomefilemodificato>
	git commit -m 'commentosullevostremodifiche'
	git push origin main
	
PER LE VOLTE SUCCESSIVE IN CUI VUOI RICONNETTERTI A QUESTO SERVER REMOTO

 1. All'interno della directory clonata digitare
	git pull 
(integrerà i cambiamenti del repository remoto 'Pianificazione-Controllo' all'interno del branch su cui lavori. Questo perchè quando riprenderai il tuo lavoro, probabilmente qualcuno del tuo gruppo avrà fatto altre modifiche)- comando per aggiornare e quindi sovrascrivere la tua copia locale del repository
(1. se non vuoi sovrascrivere la tua copia locale,sempre all'interno della cartella clonata, usa 
	git fetch
ma comunque per i nostri scopi penso sia più utile git pull
2. fare git add, git commit ecc. come sopra
