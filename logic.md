
Behaviour: 1. When a person gets too close (sonar) to the house, an alarm is sounded (piezo).

```mermaid
flowchart TD
    terminalStart([Start])
    %% Comment
    terminalEnd([End])
    thresholdSet(distanceThreshold = 30)
    setPiezoPin(piezoPin = 2)
    currentDistanceReading(distanceRead = response from Sonar)
    activatePiezo(write HIGH to piezoPin)
    deactivatePiezo(write LOW to piezoPin)

    ifDistanceLessThanThreshold{distanceRead < distanceThreshold}

    terminalStart --> thresholdSet
    thresholdSet --> setPiezoPin
    setPiezoPin --> currentDistanceReading
    currentDistanceReading --> ifDistanceLessThanThreshold
    ifDistanceLessThanThreshold --> |True| activatePiezo
    ifDistanceLessThanThreshold --> |False| deactivatePiezo
    deactivatePiezo --> terminalEnd
    activatePiezo --> terminalEnd
```

Behaviour 2: When the garage door button is pressed (button) the Door Opens (servo) and the red light is turned on (Traffic Light Red LED). There is a pause for 5 seconds, then the door is closed and the light turned off.

```mermaid
flowchart TD
    terminalStart([Start])
    terminalEnd([End])
```