# Tests sur PC (host) du firmware

Ces tests compilent les vraies sources du firmware avec un gcc natif et les relient à
`fakes.c`, qui simule le matériel : HAL SPI/GPIO, un modèle de l'ADS1118, la lecture de
l'ADS1299, les notifications BLE et le séquenceur.

Sources testées :
- `fw/Core/Src/ads1118.c` : pilote ADS1118 (clip PPG)
- `fw/STM32_WPAN/App/p2p_server_app.c` : construction du paquet EEG + PPG

Ce qui est vérifié :
- le format du paquet de 245 octets (index, 10 échantillons EEG inchangés, 2 échantillons PPG en big-endian) ;
- le calendrier SPI : conversion lancée aux échantillons EEG 0 et 5, lue aux échantillons 4 et 9 ;
- l'absence de conflit sur SPI1 (l'ADS1299 n'est jamais lu pendant que l'ADS1118 est sélectionné) ;
- aucun accès à l'ADS1118 en dehors de l'interruption DRDY, donc rien avant que l'ADS1299
  soit configuré (un accès avant `ADS1299_Init()` dérèglait l'ADS1299 sur la carte) ;
- la lecture en trame de 32 bits (donnée + relecture de la configuration) et les cas d'erreur
  (ADS1118 absent, erreur SPI au lancement ou à la lecture : valeur invalide `0x8000`) ;
- le comportement existant : aucun accès sans abonnement BLE.

## Lancer les tests

Il faut un gcc et un make natifs. Sous Windows, ceux fournis avec CLion conviennent :

```bash
cd tests/host
PATH="/c/Program Files/JetBrains/CLion 2026.1.1/bin/mingw/bin:$PATH" mingw32-make
```

Ces tests ne remplacent pas la compilation ARM dans STM32CubeIDE ni l'essai sur la carte.
Le dossier `tests/` est hors des dossiers sources du projet CubeIDE : il n'est pas compilé dans le firmware.
