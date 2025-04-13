# Heterogeni komunikacijski sustav za robotske operacije u CBRNe okruženju

### Što, zašto i kako
CBRNE incidenti koji obuhvaćaju nekontrolirano oslobađanje kemijskih, bioloških, radioloških, nuklearnih ili eksplozivnih materijala stvaraju izuzetno opasna okruženja. Takvi događaji predstavljaju ozbiljnu prijetnju ljudskom zdravlju, okolišu i infrastrukturi, često čineći područje nedostupnim ili preopasnim za ljudsku intervenciju. Multiagentski robotski sustavi nude ključnu sposobnost za izviđanje, nadzor i intervenciju u ovim kritičnim scenarijima, značajno povećavajući sigurnost osoblja. Međutim, učinkovitost ovih robotskih timova uvelike ovisi o robusnoj i pouzdanoj komunikaciji, koja je često ugrožena u specifičnim uvjetima CBRNE okruženja. 

Ovaj diplomski rad stoga istražuje i razvija heterogeni komunikacijski sustav dizajniran upravo za takve zahtjevne primjene. Sustav ima za cilj kombinirati prednosti različitih komunikacijskih tehnologija kako bi se osigurala otpornost i kontinuitet prijenosa podataka. U tu svrhu, analizirane su dostupne radio komunikacijske tehnologije s mogućnostima aktivnog umrežavanja te će se ispitati fizički sloj za prijenos podataka putem optičkog kabela za uporabu sustava u okolinama prisutnih jakih radio komunikacijskih smetnji ili namjernih ometnja aktivne radio komunikacije. Razvit će se specifične elektroničke komponente i programsko rješenje za integraciju radio i optičke komunikacije u jedinstvenu arhitekturu.  Funkcionalnost i performanse predloženog sustava ispitat će se unutar simuliranog heterogenog robotskog sustava koji oponaša operativne uvjete CBRNE incidenta, uz evaluaciju njegove efikasnosti i pouzdanosti.  Konačni cilj je stvoriti pouzdani, funkcionalni i integrirani komunikacijski sustav koji će unaprijediti sposobnosti i sigurnost robotskih timova u CBRNE operacijama. 

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

Nakon detaljne analize i usporedbe ključnih performansi dostupnih bežičnih tehnologija, odabrana je Wi-Fi HaLow tehnologija, specificirana standardom IEEE 802.11ah. Ova tehnologija koristi nelicencirani frekvencijski pojas ispod 1 GHz, tipično centralnu frekvenciju 868MHz na području Europe, što fizikalno omogućuje superioran domet signala, premašujući desetak kilometara te značajno bolju penetraciju kroz građevinske materijale i druge prepreke u usporedbi s tehnologijama koje koriste zasićeni 2.4 GHz pojas, poput Bluetooth Low Energy ili Zigbee bežičnih tehnologija.

Wi-Fi HaLow nudi iznimno širok i prilagodljiv raspon brzina prijenosa podataka, skalirajući od minimalnih 150 kbps za energetski učinkovite senzorske aplikacije do maksimalnih 86.7 Mbps korištenjem kanala širine 16 MHz i najviše modulacijsko-kodne sheme (MCS), čime se osigurava kapacitet za prijenos raznolikog prometa od osnovne telemetrije i kontrolnih signala do video prijenosa visoke razlučivosti s robotskih platformi. Robusnost komunikacije u uvjetima višestaznog širenja signala, karakterističnog za kompleksna urbana ili industrijska CBRNe okruženja, postiže se primjenom napredne OFDM (Orthogonal Frequency-Division Multiplexing) modulacije uz podršku za adaptivne sheme poput BPSK, QPSK, te 16/64/256 QAM, optimizirajući spektralnu efikasnost i otpornost na smetnje. Iako je protokol dizajniran s mehanizmima za nisku potrošnju energije, uključujući Target Wake Time (TWT) koji omogućuje uređajima dugotrajne periode spavanja, stvarna energetska bilanca na mobilnoj robotskoj platformi ovisit će o konfiguraciji izlazne snage RF predajnika, odabranoj MCS shemi i radnom ciklusu komunikacije, no sami temelji protokola omogućuju znatno efikasnije upravljanje energijom od tradicionalnih Wi-Fi standarda. 
 
Sigurnost komunikacijskog kanala, ključna za integritet kontrolnih naredbi i povjerljivost prikupljenih podataka, osigurana je implementacijom suvremenog WPA3 sigurnosnog standarda, koji uključuje poboljšane mehanizme autentifikacije (poput SAE - Simultaneous Authentication of Equals) i snažnu enkripciju. Značajna operativna prednost je nativna podrška za OTA (Over-the-Air) ažuriranja programskog koda (firmware-a), što omogućuje daljinsko održavanje, implementaciju sigurnosnih zakrpa i nadogradnju funkcionalnosti robotskih jedinica bez potrebe za fizičkim pristupom, što je neprocjenjivo u potencijalno kontaminiranim zonama. Za razliku od komercijalnih naplatnih LPWAN tehnologija (NB-IoT) ili nekih drugih LPWAN mreža (Sigfox, komercijalni LoRaWAN), Wi-Fi HaLow ne nameće potrebu za plaćanjem mjesečne pretplate za uslugu prijenosa podataka, što je izuzetno važno za integraciju. Odabrani protokol pruža punu podršku za standardni TCP/IP protokolni složaj, omogućujući direktnu i jednostavnu integraciju robotskih sustava u postojeće IP mreže i komunikaciju s centralnim kontrolnim sustavima putem interneta ili lokalnih mreža. Za konkretnu implementaciju, razvoj aplikacijskog sloja i eksperimentalnu validaciju performansi u okviru ovog rada, koristi se Newracom NRC7394 Evaluation Kit (EVK). Ovaj EVK predstavlja cjelovitu razvojnu platformu koja integrira NRC7394 System-on-Chip (SoC), RF front-end sklopovlje, antenski priključak te je baziran na Raspberry Pi 4 SBC (Single board computer) kontroleru, omogućujući detaljno testiranje propusnosti, latencije, dometa i energetske potrošnje u realnim uvjetima primjene specifičnim za CBRNe scenarije.

(O newracom EVK, sustav, setup, topologija pic)


### Optička veza s dronom
malo o media konverterima
puno o tension sustavu


### Napajanje
? shema napajanja (BEC+sustavi?)








-----


ROS2:
- fixed, blocking funckije nisu bitne za sad (cin) jer ih neće bit u final stvari

RPI:
* WiFi AP - DONE
* Static IP na Ethernetu - DONE
* 24.04, ROS, etc... - DONE

NEWRACOM:
*  NRC7394 - NOT DONE, needs equipment

TENSION:
* Load Cell Tension sys - DONE, printed
* Load Cell programing - NO, needs equipment
* ovaj vrag za namatanje - DONE, printed


GIT:
* update

diplomski na overleafu:
https://www.overleaf.com/read/jkqhyxytmrxt#3c8907
