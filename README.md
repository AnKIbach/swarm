# autopilot_ROSpkg

Autopilot og kommunikasjon til bacheloroppgave med båter

Hovedprogrammene er Autopilot.py, Behaviour.py og Communication.py

'' Mappestruktur: Base_files - filer til GUI for base Multicast - filer til å sende og motta data med multicast, mest brukt av Communication.py ROS_operators - operatører og klasser for bruk av ROS, dvs innhenting og videresending av data. Source_pictures - kun bilder til bruk i testing av Tkinter og Pygame Old_files - filer som er brukt tidligere, men i nåverende utgivelse er "obsolete"

Mål - det skal kun være de tre hovedprogrammene som ligger i parent-directoriet Autopilot, mens alle filer ligger i mapper under. ''

'' Arbeides med nå? På vei ut på vannet ''

'' Setup på ny pc:

https://code.visualstudio.com/docs/python/python-tutorial !!!

    last ned python

    bruk command pallette med kommandoer shift-ctrl-p : python: select interpreter og python: select..

    last ned pylint - kommer en pop-up, om ikke bruk pip pylint

    Kjør opp en virtual env med: py -3 -m .venv env & src\scripts\activate 4.5. Om det gir feil må du skrive inn: Set-ExecutionPolicy Unrestricted i powerShell som admin

    i virtuelle så kjører man pip og laster ned andre pakker etter behov 5.5. Pr nå brukes: pygame pyserial pyfirmata

    Plassering av mappen - vær bevisst på at programmer som ligger i mappen ROS_operators krever at prosjektmappen kjøres i en catkin workspace på en PC med ROS og MAVROS installert ''
