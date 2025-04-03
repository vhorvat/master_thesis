# Heterogeni komunikacijski sustav za robotske operacije u CBRNe okruženju

### Što, zašto i kako
CBRNE incidenti koji obuhvaćaju nekontrolirano oslobađanje kemijskih, bioloških, radioloških, nuklearnih ili eksplozivnih materijala stvaraju izuzetno opasna okruženja. Takvi događaji predstavljaju ozbiljnu prijetnju ljudskom zdravlju, okolišu i infrastrukturi, često čineći područje nedostupnim ili preopasnim za ljudsku intervenciju. Multiagentski robotski sustavi nude ključnu sposobnost za izviđanje, nadzor i intervenciju u ovim kritičnim scenarijima, značajno povećavajući sigurnost osoblja. Međutim, učinkovitost ovih robotskih timova uvelike ovisi o robusnoj i pouzdanoj komunikaciji, koja je često ugrožena u specifičnim uvjetima CBRNE okruženja. Ovaj diplomski rad stoga istražuje i razvija heterogeni komunikacijski sustav dizajniran upravo za takve zahtjevne primjene. Sustav ima za cilj kombinirati prednosti različitih komunikacijskih tehnologija kako bi se osigurala otpornost i kontinuitet prijenosa podataka. U tu svrhu, analizirane su dostupne radio komunikacijske tehnologije s mogućnostima aktivnog umrežavanja te će se ispitati fizički sloj za prijenos podataka putem optičkog kabela za uporabu sustava u okolinama prisutnih jakih radio komunikacijskih smetnji ili namjernih ometnja aktivne radio komunikacije. Razvit će se specifične elektroničke komponente i programsko rješenje za integraciju radio i optičke komunikacije u jedinstvenu arhitekturu.  Funkcionalnost i performanse predloženog sustava ispitat će se unutar simuliranog heterogenog robotskog sustava koji oponaša operativne uvjete CBRNE incidenta, uz evaluaciju njegove efikasnosti i pouzdanosti.  Konačni cilj je stvoriti pouzdani, funkcionalni i integrirani komunikacijski sustav koji će unaprijediti sposobnosti i sigurnost robotskih timova u CBRNE operacijama. 

### Shematski prikaz predloženog rješenja
![Shema sustava](docs/images/shema.png)

### Bežični komunikacijski sustav
Analizirane su tehničke specifikacije komercijalno dostupnih bežičnih komunikacijskih sustava u kategoriji korištenih frekvencija, brzine prijenosa, dometa, modulacije, potrošnje energije, sigurnostimogućnosti OTA (Over the Air) ažuriranja programskih kodova RF (Radio Frequency) sklopovlja, potrebne pretplate na uslugu prijenosa podataka, podrške za TCP/IP protokolni složaj, dostupnih topologija te otvorenost standarda na kojem se temelji njihovo funkcioniranje. Rezultati su dani u sljedećoj tablici:


| Atributi                   | Wi-Fi HaLow                       | Bluetooth Low Energy | Z-Wave             | Zigbee                   | Wi-SUN                          | Sigfox                     | LoRaWAN                    | NB-IoT                    |
| :------------------------- | :-------------------------------- | :------------------- | :----------------- | :----------------------- | :------------------------------ | :------------------------- | :------------------------- | :------------------------ |
| **Frekvencija**            | Ispod 1 GHz                       | 2.4 GHz              | Ispod 1 GHz        | 2.4 GHz / Ispod 1 GHz    | Ispod 1 GHz                     | Ispod 1 GHz                | Ispod 1 GHz                | Licencirano               |
| **Brzina prijenosa (bps)** | 150 k - 86.7 M⁸                   | 125 k - 2 M          | 9.6 k - 100 k      | 250 k                    | 6.25 k - 800 k (50 k zadano)  | 100 ili 600                | 300 - 27 k                 | 20 k - 127 k              |
| **Domet (m)**              | > 1 k                             | < 100                | < 30               | < 20                     | < 1 k                           | < 40 k                     | < 10 k                     | < 10 k                    |
| **Modulacija**             | OFDM preko BPSK, QPSK, 16/64/256 QAM | GFSK                 | GFSK               | BPAK/OQPSK               | MR-FSK / MR-OFDM / MR-OQPSK     | DBPSK/GFSK                 | CSS                        | QPSK                      |
| **Trajanje baterije**      | Godine                            | Godine               | Godine             | Godine                   | Godine                          | Godine                     | Godine                     | Godine                    |
| **Sigurnost**              | WPA3                              | 128-bit AES u CCMode | Sigurnost 2 (S2)   | 128-bit AES u CCMode     | IEEE 802.1X                     | Sigurnost na razini sesije | 128-bit AES u CCMode     | 3GPP sigurnost            |
| **OTA ažuriranja firmvera**| Podržava                          | Podržava             | -                  | -                        | -                               | -                          | -                          | -                         |
| **Potrebna pretplata**     | Ne                                | Ne                   | Ne                 | Ne                       | Ne                              | Da                         | Da                         | Da                        |
| **TCP/IP (internet)**      | Podržava                          | -                    | -                  | -                        | -                               | -                          | -                          | -                         |
| **Mrežna topologija**      | Zvijezda / Releji                 | P2P* / Mreža (Mesh)  | Mreža (Mesh)       | Mreža (Mesh)             | Mreža (Mesh)                    | Zvijezda                   | Zvijezda                   | Zvijezda                  |
| **Otvoreni standard**      | IEEE 802.11ah                     | Bluetooth SIG        | Vlasnički          | IEEE 802.15.4            | IEEE 802.15.4g                  | Vlasnički                  | Vlasnički                  | 3GPP LTE Cat-NB1/NB2      |

(izvor WiFi Aliance..)

### WiFi HaLow

ROS2 packages:
* fix issue on exit signal
* fix code comments, check the current (mA) values and scalling 

RPI:
* everything

GIT:
* update git ignore list za buildane stvari
* create docs on building the code 
