# README `main.m`

L'obiettivo del file e' leggere in tempo reale la seriale del tag UWB, trasformare le misure in una stima di posizione 2D e aggiornare un filtro che mostri il tracking del tag nello spazio.

## Scopo del file

`main.m` fa quattro cose principali:

1. legge i round UWB dal tag via seriale;
2. converte i messaggi seriali in misure di distanza verso le ancore;
3. seleziona le misure piu' credibili;
4. usa un filtro di stima per ottenere una posizione 2D piu' stabile del semplice ranging grezzo.

In altre parole:

- il firmware del tag fa il ranging e un primo filtraggio radio;
- `main.m` fa il filtraggio logico/geometrico e la stima di posizione.

## Input che arrivano dal tag

Il tag stampa righe del tipo:

```text
RANGING MEAS [round] [tag->anchor] distanza_mm QUAL qual_x10 pream FLAG LOS/NLOS
```

e una riga finale di riepilogo per round:

```text
[round] round: X meas | Y LOS | Z NLOS | T timeout | D dup
```

Questo e' importante per una ragione precisa:

- non vogliamo usare misure sparse e mescolate nel tempo;
- vogliamo usare **misure dello stesso round**, cioe' quasi simultanee.

Per questo `main.m` legge i dati round per round, invece di prendere la prima distanza disponibile e usarla subito.

## Struttura generale del file

Il flusso di `main.m` e':

1. imposta i parametri;
2. carica la mappa delle ancore;
3. carica varianze e bias offline;
4. apre la seriale;
5. aspetta un primo round valido per inizializzare `x0`;
6. entra in un loop infinito:
   - legge un round;
   - filtra le misure;
   - predice lo stato;
   - aggiorna il filtro;
   - aggiorna il grafico.

## Parametri iniziali

I parametri principali sono:

- `min_anchor_ekf = 3`
- `min_anchor_update = 2`
- `max_anchor_ekf = 8`
- `target_anchor_ekf = 3`
- `min_qual_x10 = -60`
- `min_pream_count = 100`
- `dT = 0.5`
- `sigma_pos = 0.5`
- `sigma_theta = 0.3`
- `Q = diag([sigma_pos^2, sigma_pos^2, sigma_theta^2])`
- `min_heading_step = 0.12`
- `vel_smoothing = 0.70`

### Perche' questi parametri esistono

#### `min_anchor_ekf = 3`

In 2D, con meno di 3 ancore, la geometria diventa troppo debole o ambigua.

Per questo il codice usa almeno 3 ancore per inizializzare `x0`.

Questo vincolo e' tenuto severo in inizializzazione, perche' partire con una posizione sbagliata rovina tutto il tracking successivo.

#### `min_anchor_update = 2`

Durante il tracking realtime il codice e' un po' piu' permissivo e prova ad aggiornare anche con 2 ancore buone.

La motivazione e' pratica:

- nei test reali ci sono molti round con solo 2 ancore affidabili;
- se richiedessimo sempre 3 ancore per ogni update, il filtro passerebbe troppo tempo in sola predizione;
- usare 2 ancore nel runtime e' un compromesso per non perdere completamente il tag nei tratti piu' difficili.

Questo non vale per l'inizializzazione, dove invece si preferisce restare piu' severi.

#### `max_anchor_ekf = 8`

Non si usano tutte le ancore disponibili senza limite.

Motivo:

- troppe misure possono introdurre ancore marginali o rumorose;
- il costo combinatorio della selezione cresce;
- non sempre "piu' ancore" vuol dire "migliore geometria".

Quindi si mette un tetto ragionevole.

#### `target_anchor_ekf = 3`

Nella configurazione che sta funzionando meglio adesso, il filtro prova a lavorare con 3 ancore selezionate bene.

Perche' questa scelta e' risultata migliore di `4` nelle prove reali:

- 3 sono il minimo geometrico per il tracking 2D;
- con 4 ancore il filtro tendeva piu' spesso a includere una misura marginale;
- nel setup reale i round molto ricchi non sono cosi' frequenti, quindi chiedere 4 ancore ben utilizzabili rendeva il filtro meno continuo;
- usare 3 ancore ma sceglierle bene si e' rivelato piu' stabile.

Questo non vuol dire che `3` sia sempre meglio di `4` in assoluto.

Vuol dire che, nel setup che stai usando adesso, `target_anchor_ekf = 3` e' la baseline pratica migliore.

#### `min_qual_x10` e `min_pream_count`

Sono soglie lato MATLAB aggiunte sopra al filtro del firmware.

Perche' non fidarsi solo del tag:

- il tag fa un primo filtro radio, utile per togliere il peggio;
- MATLAB deve essere un secondo livello di controllo, piu' conservativo.

Quindi la scelta e':

- tag: filtro radio rapido e locale;
- MATLAB: filtro piu' "logico", adatto alla stima.

#### `dT = 0.5`

Vale il periodo del round del tag.

Serve per:

- stimare una velocita' istantanea;
- definire la dinamica del filtro.

#### `Q`

`Q` e' il rumore di processo del modello.

In questa versione il rumore di processo e' scritto in forma piu' leggibile:

- `sigma_pos = 0.5`
- `sigma_theta = 0.3`
- `Q = diag([sigma_pos^2, sigma_pos^2, sigma_theta^2])`

Questa forma e' preferita a una matrice numerica "secca" perche':

- rende piu' chiaro il significato fisico dei parametri;
- separa l'incertezza traslazionale da quella angolare;
- rende piu' semplice il tuning durante i test.

Se `Q` e' troppo piccolo:

- il filtro si fida troppo della predizione;
- segue male i cambiamenti reali.

Se `Q` e' troppo grande:

- il filtro si fida troppo delle misure;
- la traiettoria diventa nervosa.

Questa matrice e' quindi un compromesso tra stabilita' e reattivita'.

In pratica:

- `sigma_pos` regola quanto il filtro accetta che la posizione possa cambiare tra due round;
- `sigma_theta` regola quanto l'orientamento possa cambiare anche senza una misura diretta dedicata.

#### `min_heading_step` e `vel_smoothing`

Questi due parametri servono a stabilizzare la parte dinamica del filtro.

`min_heading_step` evita di aggiornare l'heading quando lo spostamento stimato e' troppo piccolo.

Motivo:

- da fermo o quasi fermo, una stima di heading ricavata da spostamenti minuscoli e' dominata dal rumore;
- e' meglio lasciare l'orientamento quasi fermo che farlo ruotare a caso.

`vel_smoothing` smussa la velocita' stimata tra un round e il successivo.

Motivo:

- la velocita' grezza ricavata da due stime consecutive e' molto rumorosa;
- se la usassimo direttamente, il modello dinamico reagirebbe in modo troppo nervoso.

## Caricamento mappa e ancore

Il file usa:

- `DEPT_evb1000_map.csv`
- `tracking_node_ids = [108, 113:119, 121:154]`

### Perche' si filtra la mappa

La mappa puo' contenere piu' nodi di quelli davvero usati nel tracking.

Se MATLAB usasse nodi non presenti nel firmware:

- vedrebbe indirizzi non coerenti;
- assocerebbe misure a coordinate sbagliate;
- la stima diventerebbe inconsistente.

Per questo si tiene solo il set di ancore realmente previsto dal setup.

### Perche' si estrae `addr_short`

Le righe seriali del tag riportano l'indirizzo corto delle ancore, non il `NodeId`.

Quindi MATLAB deve mappare:

- indirizzo seriale UWB
- coordinate geometriche
- `node_id`

Questa associazione e' fondamentale, altrimenti la distanza verrebbe assegnata all'ancora sbagliata.

## Uso di varianze e bias offline

Il file carica `variance_per_node.csv` e costruisce:

- `noise_map`
- `bias_map`

### Perche' usiamo la varianza per ancora

Non tutte le ancore sono ugualmente affidabili.

Alcune hanno:

- piu' rumore;
- piu' dispersione;
- misure storicamente peggiori.

Usare una varianza per ancora permette di:

- pesare meno le ancore rumorose;
- rendere il filtro meno sensibile a quelle ancore.

### Perche' usiamo anche il bias

La varianza misura la dispersione, ma non l'errore sistematico.

Se un'ancora tende a sovrastimare o sottostimare sempre di una certa quantita', la sola varianza non basta.

Per questo il codice corregge ogni misura con:

- `dist_m = misura - bias_ancora`

Questa e' una correzione pratica basata su una calibrazione offline.

### Limite di questa scelta

Il bias stimato offline non e' garantito identico in ogni condizione realtime.

Quindi:

- spesso aiuta;
- ma non va considerato una verita' assoluta.

## Modello dinamico

Il filtro usa uno stato:

- `x`
- `y`
- `theta`

e un modello tipo unicycle.

### Perche' usare un modello dinamico

Le misure UWB non arrivano perfette a ogni round.

Se usassimo solo la trilaterazione pura:

- la stima salterebbe molto;
- nei round con misure deboli non avremmo continuita'.

Il modello dinamico serve a dire:

- "se non ho una misura perfetta, so comunque che il tag non puo' teletrasportarsi".

### Perche' `omega_inst = 0`

In questa versione non c'e' una vera misura di velocita' angolare.

Quindi si preferisce:

- non inventare una rotazione;
- lasciare che l'orientamento sia soprattutto una variabile di supporto al modello.

E' una scelta semplice e robusta, non la piu' sofisticata.

## Inizializzazione

All'avvio il codice aspetta un primo round valido:

```matlab
[init_anchors, init_dists, init_noise, init_round] = collect_round_measurements(...)
```

Poi esegue una trilaterazione lineare per ottenere `x0`.

### Perche' inizializzare dal primo round completo

Questa versione e' cosi per una ragione pratica:

- e' piu' semplice;
- parte piu' velocemente;
- in alcune prove reali si e' comportata meglio di versioni piu' "ambiziose" ma meno stabili.

Quindi qui e' stata privilegiata la robustezza pratica rispetto alla complessita' teorica.

### Perche' si usa `pinv` e non `inv`

Le matrici possono diventare quasi singolari se:

- le ancore sono mal distribuite;
- la geometria e' quasi degenere.

`pinv` e' piu' robusta numericamente di `inv` in questi casi.

## Lettura dei round seriali

La funzione chiave e':

- `collect_round_measurements(...)`

Questa funzione continua a leggere finche' non trova un round con abbastanza misure radio-buone.

### Perche' scartare round vuoti o troppo poveri

Un round con:

- 0 ancore;
- 1 ancora;
- 2 ancore;

non aiuta davvero la stima 2D.

Meglio aspettare un round piu' utile che aggiornare il filtro con informazione debole o ambigua.

## Parsing delle misure

`read_single_round(...)` e `parse_ranging_meas(...)` servono a trasformare le righe di testo in una struttura MATLAB.

Ogni misura contiene:

- indirizzo ancora
- distanza
- qualita'
- preamble count
- flag `LOS/NLOS`
- rumore associato
- bias associato
- coordinate dell'ancora

### Perche' fare una struttura completa per misura

Perche' cosi' ogni round resta autosufficiente:

- non abbiamo solo la distanza;
- abbiamo anche contesto radio e geometrico.

Questo permette al codice di prendere decisioni migliori nel passo successivo.

## Deduplica per ancora

Se nello stesso round arrivano due misure dalla stessa ancora, il codice tiene quella migliore.

La logica e':

1. `LOS` meglio di `NLOS`
2. `QUAL` piu' alta meglio di `QUAL` piu' bassa
3. `pream` piu' alto meglio di `pream` piu' basso

### Perche' questa scelta

Non vogliamo dare due volte peso alla stessa ancora.

Inoltre, se esistono due versioni della stessa misura, conviene usare quella con qualita' radio migliore.

## Primo filtro: radio-good anchors

In `collect_round_measurements(...)` viene costruita una maschera:

- `los_flags`
- `qual_values >= min_qual_x10`
- `pream_values >= min_pream_count`

### Perche' questo filtro avviene prima della stima

Perche' e' un filtro economico e molto informativo.

Se una misura e' gia' radio-povera:

- non ha senso portarla fino alla trilaterazione;
- rischia solo di inquinare la stima.

Quindi questo e' il primo sbarramento.

## Scoring delle misure

Dopo il filtro radio, il codice costruisce uno score:

```matlab
score = qual + 0.05 * pream - 20 * noise
```

### Perche' fare uno score e non usare una sola soglia

Una soglia dice solo "dentro/fuori".

Uno score permette di ordinare le misure buone tra loro:

- qualita' migliore;
- preambolo migliore;
- rumore minore.

Questo e' utile quando ci sono piu' ancore valide di quelle che vogliamo usare.

## Predizione

Nel loop, prima si predice:

```matlab
[x_pred, P_pred] = predict_step(...)
```

### Perche' predire prima di aggiornare

E' il meccanismo base del Kalman:

- predizione = cosa mi aspetto prima di vedere il nuovo round
- update = correzione con le misure reali

Senza predizione, il filtro diventerebbe una semplice correzione statica da round a round.

## Selezione del sottoinsieme di ancore

Il codice non usa automaticamente tutte le ancore radio-buone.

Usa:

- `select_anchor_subset(...)`

per scegliere un sottoinsieme preferito.

### Perche' non usare tutte le ancore buone

Perche' anche tra ancore "buone" ce ne possono essere alcune:

- ridondanti;
- mal distribuite;
- quasi collineari;
- con residui peggiori rispetto alla predizione.

Questa funzione cerca un compromesso tra:

- geometria
- copertura angolare
- rumore
- coerenza con la posizione predetta

### Perche' usare la geometria

Tre ancore quasi allineate possono avere segnale buono ma dare una trilaterazione pessima.

Per questo il codice non guarda solo la radio, ma anche:

- area del triangolo massimo;
- copertura angolare rispetto a `x_pred`.

## Gating sui residui

Dopo la selezione del subset, il codice confronta:

- distanza misurata
- distanza attesa dalla posizione predetta

e costruisce un residuo.

### Perche' il residuo e' importante

Una misura puo' essere:

- radio-buona;
- ma geometricamente incoerente col round corrente.

Il residuo serve proprio a intercettare questo caso.

### Perche' la soglia non e' fissa

La soglia usa:

- rumore dell'ancora;
- traccia della covarianza sulla posizione;
- velocita' stimata.

Questo perche':

- se il filtro e' incerto, una soglia troppo stretta sarebbe ingiusta;
- se il tag si muove, una piccola tolleranza in piu' puo' essere sensata.

## Rescue dei round quasi validi

Se dopo il gating restano meno di 3 misure, il codice prova a salvare il round prendendo le ancore col residuo piu' basso.

### Perche' esiste il rescue

Senza rescue, basta una sola ancora leggermente fuori soglia per buttare via un round che magari era quasi buono.

Il rescue e' stato introdotto per:

- ridurre il numero di round persi;
- mantenere stabilita' senza aprire troppo le soglie globali.

E' quindi una soluzione di compromesso:

- non accettiamo tutto;
- ma non buttiamo via un round per una sola misura borderline.

## Check di geometria finale

Anche dopo radio filter e residue gating, il codice fa ancora:

- `max_triangle_area(active_anchors_ekf) < 0.5`

### Perche' questo controllo finale

Perche' si possono avere 3 ancore "buone" ma quasi sulla stessa linea.

In quel caso:

- l'algebra ti darebbe comunque una posizione;
- ma fisicamente sarebbe poco osservabile e molto fragile.

Quindi meglio saltare l'update che introdurre una correzione ingannevole.

## Update del filtro

Se il round passa tutti i controlli, il codice fa:

1. costruzione di un update diretto sulle distanze;
2. linearizzazione locale della misura rispetto allo stato predetto;
3. update tipo Kalman.

### Perche' questa versione usa un update diretto sulle distanze

In questa versione il runtime non passa piu' obbligatoriamente da una trilaterazione completa ad ogni round.

Usa invece un update sulle distanze stesse, linearizzato nello stato predetto.

La ragione e' pratica:

- quando ci sono solo 2 ancore buone, una trilaterazione completa diventa troppo fragile o non e' nemmeno ben definita;
- l'update diretto sulle distanze permette al filtro di usare anche informazione parziale;
- questo aiuta a non "perdere" il tag nei tratti in cui la copertura radio peggiora.

Non e' la formulazione piu' sofisticata possibile, ma ha vantaggi concreti:

- e' veloce;
- e' facile da debuggare.

La trilaterazione lineare resta comunque usata in inizializzazione, dove avere una posa geometrica iniziale ha ancora senso.

### Perche' il filtro puo' comunque saltare tanti round

Perche' questa implementazione privilegia ancora la stabilita' rispetto alla continuita'.

Se il round non convince, il codice preferisce:

- non correggere;
- usare la predizione.

E' una scelta conservativa.

## Plot realtime

Il grafico mostra:

- tutte le ancore in blu;
- la traiettoria stimata in verde;
- la posizione attuale in rosso;
- l'heading stimato;
- le ancore attive del round cerchiate in verde.

### Perche' mostrare anche le ancore attive

Perche' nel debug del tracking e' fondamentale capire:

- non solo dove sta andando la stima;
- ma anche **quali ancore** stanno guidando quell'update.

Molti problemi di tracking non nascono dal filtro in se', ma da:

- poche ancore;
- ancore tutte da un lato;
- geometria debole.

Visualizzare le ancore attive rende questi problemi evidenti.

## Filosofia complessiva del codice

`main.m` segue questa idea:

- il firmware del tag fa un pre-filtro radio rapido;
- MATLAB applica un filtro piu' severo;
- meglio perdere qualche round che aggiornare con misure molto sporche;
- meglio usare poche ancore ma ben scelte che tante ancore incoerenti;

## Configurazione pratica attuale

Nelle ultime prove reali, la configurazione che ha dato il comportamento piu' convincente e':

- `target_anchor_ekf = 3`
- `min_anchor_update = 2`
- `min_pream_count = 100`
- `min_qual_x10 = -60`
- `Q = diag([0.5^2, 0.5^2, 0.3^2])`
- `min_heading_step = 0.12`
- `vel_smoothing = 0.70`

Perche' questa configurazione e' diventata la baseline:

- l'inizializzazione cade nella zona giusta piu' spesso;
- la stima resta piu' raccolta vicino alla posizione reale;
- nei tratti in cui restano solo 2 ancore buone il filtro riesce ancora ad aggiornarsi;
- i round persi rimangono presenti, ma non dominano il comportamento del filtro;
- forzare `4` ancore, nel tuo setup, tendeva a peggiorare piu' che migliorare.

Questa sezione non descrive una regola teorica generale.

Descrive la configurazione che, al momento, ha funzionato meglio nelle prove sul campo.

## Limiti attuali

Questa versione ha anche limiti noti:

- l'inizializzazione dal primo round puo' essere sensibile;
- il modello unicycle e' semplificato;
- l'heading non e' veramente osservato da una misura dedicata;
- anche con l'update diretto sulle distanze, se i round hanno solo 1 ancora buona il filtro non puo' correggere davvero;
- se i round con 3 ancore buone sono pochi, nessun tuning MATLAB puo' fare miracoli.

## Quando conviene cambiare approccio

Se in futuro vorrai una versione piu' precisa durante il moto, le direzioni sensate sono:

- inizializzazione su piu' round;
- multilaterazione non lineare robusta;
- modello dinamico cartesiano `x, y, vx, vy`;
- uso piu' esplicito dei bias per ancora;
- logging strutturato dei round accettati/scartati.
