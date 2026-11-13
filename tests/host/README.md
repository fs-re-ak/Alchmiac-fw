# Tests sur PC (host) du firmware

Ces tests compilent les vraies sources du firmware avec un gcc natif et les relient à
`fakes.c`, qui simule le matériel : HAL SPI/GPIO, un modèle de l'ADS1118, la lecture de
l'ADS1299, les notifications BLE et le séquenceur.

Sources testées :
- `fw/Core/Src/ads1118.c` : pilote ADS1118 (clip PPG)
- `fw/STM32_WPAN/App/p2p_server_app.c` : construction du paquet EEG + PPG

Deux exécutables :
- `test_ppg` : le firmware normal ;
- `test_ble_pattern` : le firmware compilé avec le motif de test BLE (`BLE_TEST_PATTERN=1`, voir plus bas).

Ce qui est vérifié :
- le format du paquet de 245 octets (index, 10 échantillons EEG inchangés, 2 échantillons PPG en big-endian) ;
- le calendrier SPI : conversion lancée aux échantillons EEG 0 et 5, lue aux échantillons 4 et 9 ;
- l'absence de conflit sur SPI1 (l'ADS1299 n'est jamais lu pendant que l'ADS1118 est sélectionné) ;
- aucun accès à l'ADS1118 en dehors de l'interruption DRDY, donc rien avant que l'ADS1299
  soit configuré (un accès avant `ADS1299_Init()` dérèglait l'ADS1299 sur la carte) ;
- la lecture en trame de 32 bits (donnée + relecture de la configuration) et les cas d'erreur
  (ADS1118 absent, erreur SPI au lancement ou à la lecture : valeur invalide `0x8000`) ;
- le comportement existant : aucun accès sans abonnement BLE.

## Motif de test BLE

Pour valider le transfert BLE et le décodage côté logiciel, le firmware peut remplacer les
données par des compteurs 16 bits juste avant l'envoi (`BLE_TEST_PATTERN`, désactivé par
défaut, décrit en tête de `p2p_server_app.c`). L'ADS1299 et l'ADS1118 restent lus : la
cadence et le trafic SPI sont ceux du firmware normal.

| Donnée | Octets envoyés | Valeur décodée attendue |
|---|---|---|
| EEG, canaux 0 à 6 (int24 big-endian) | `[c, n >> 8, n & 0xFF]` | `c * 65536 + n` |
| EEG, canal 7 | `[0xFF, n >> 8, n & 0xFF]` | `n - 65536` (négatif) |
| PPG, 2 par paquet (int16 big-endian) | `n` des échantillons EEG 4 et 9 | `(int16) n` |
| IMU, 9 champs (int16 little-endian) | `m + i` | `(int16) (m + i)` |

`n` avance de 1 par échantillon EEG (250 Hz), `m` de 1 par paquet IMU (50 Hz). Les deux
partent de `0xFF00` au démarrage : premier retour à 0 après environ 1 s (EEG), puis toutes
les 262 s. L'index de paquet (0 à 127) est inchangé.

`test_ble_pattern` décode chaque paquet produit par le vrai code sur un cycle 16 bits
complet et vérifie l'ordre des octets, le signe, les retours à 0, et que l'acquisition
réelle tourne toujours. `BLE_PATTERN_EEG_DUMP=<fichier>` et `BLE_PATTERN_IMU_DUMP=<fichier>`
enregistrent les premiers paquets bruts, pour vérifier un décodeur avec la vraie sortie du
firmware.

Pour flasher le motif sur une carte : passer `#define BLE_TEST_PATTERN 0` à `1` dans
`p2p_server_app.c` (ou ajouter le symbole `BLE_TEST_PATTERN=1` dans les options du
préprocesseur de CubeIDE), puis recompiler. La compilation affiche un avertissement tant
que le motif est actif. **Ne jamais flasher ce firmware sur un casque utilisé pour des
mesures, ni le commiter avec le motif actif.**

## Lancer les tests

Il faut un gcc et un make natifs. Sous Windows, ceux fournis avec CLion conviennent :

```bash
cd tests/host
PATH="/c/Program Files/JetBrains/CLion 2026.1.1/bin/mingw/bin:$PATH" mingw32-make
```

Ces tests ne remplacent pas la compilation ARM dans STM32CubeIDE ni l'essai sur la carte.
Le dossier `tests/` est hors des dossiers sources du projet CubeIDE : il n'est pas compilé dans le firmware.
