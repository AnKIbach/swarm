# _Swarm

Autopilot, kommunikasjon og behaviour til bacheloroppgave med båter

Hovedprogrammene er Autopilot.py, Behaviour.py og Communication.py

''
Mappestruktur: 4 hovedmapper for de aktuelle programmene som skal kjøre og deres undermoduler

- Autopilot - inneholder alle moduler og ROS_operators for at Autopilot.py skal fungere og publisere (Bruker ROS)
- Communication - inneholder alle moduler og Multicast for at Boat_TX og -RX skal fungere og caste (Bruker ROS)
- Behaviour - inneholder alle moduler og ... for at Behaviour.py skal fungere og publisere (Bruker ROS)

- GCS - inneholder alle moduler og mapper for at GroundControlStationen skal fungere (Work in progress uten ROS)
''

'' Arbeides med nå? Test med flere båter samtidig''

'' Setup på ny pc:''

https://code.visualstudio.com/docs/python/python-tutorial !!!

    last ned python

    bruk command pallette med kommandoer shift-ctrl-p : python: select interpreter og python: select..

    last ned pylint - kommer en pop-up, om ikke bruk pip pylint

    Kjør opp en virtual env med: py -3 -m .venv env & src\scripts\activate 4.5. Om det gir feil må du skrive inn: Set-ExecutionPolicy Unrestricted i powerShell som admin

    i virtuelle så kjører man pip og laster ned andre pakker etter behov 5.5. Pr nå brukes: pygame pyserial pyfirmata

    Plassering av mappen - vær bevisst på at programmer som ligger i mappen ROS_operators krever at prosjektmappen kjøres i en catkin workspace på en PC med ROS og MAVROS installert ''
