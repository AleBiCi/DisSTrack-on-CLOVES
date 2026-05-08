# Real-Time Tracking Firmware

Questo folder contiene i due firmware flashati sulle schede EVB1000 per il tracking realtime:

- `rng-init-all.bin`: firmware del `tag`
- `rng-resp.bin`: firmware delle `ancore`

Le sorgenti principali sono:

- `rng-init-all.c`
- `rng-resp.c`
- `generate_and_build_all.py`
- `rng-support.h`
- `rng-support.c`

## Idea generale

Il sistema lavora a round.

1. Il `tag` invia un pacchetto `INIT` in broadcast.
2. Ogni `ancora` che riceve l'`INIT` risponde nel proprio slot temporale.
3. Il `tag` ascolta tutta la finestra del round, raccoglie le risposte, calcola la distanza e valuta la qualità del segnale.
4. Il `tag` manda tutto su seriale verso `MATLAB`.

In questo modo:

- il tag è la scheda collegata via USB al PC;
- le ancore non parlano tra loro;
- `MATLAB` legge solo la seriale del tag.

## File `rng-init-all.c`

Questo è il firmware del `tag`.

### Cosa fa

- inizializza il DW1000;
- ogni `0.5 s` avvia un nuovo round di ranging;
- costruisce e trasmette un pacchetto `INIT` broadcast;
- apre una finestra RX abbastanza lunga da ricevere tutte le `RESP` delle ancore;
- per ogni risposta valida:
  - legge i timestamp SS-TWR;
  - calcola il `time of flight`;
  - converte il risultato in distanza;
  - legge le metriche di qualità arrivate dall'ancora;
  - decide se la misura è `LOS` o `NLOS`;
  - tiene una sola misura per ancora nel round;
- stampa su seriale le misure del round.

### Come calcola la distanza

Il tag usa il classico schema `SS-TWR`:

- `init_tx_ts`: quando il tag ha trasmesso `INIT`
- `init_rx_ts`: quando l'ancora ha ricevuto `INIT`
- `resp_tx_ts`: quando l'ancora ha trasmesso `RESP`
- `resp_rx_ts`: quando il tag ha ricevuto `RESP`

Da questi quattro tempi ricava il `TOF` e poi la distanza:

- `dist_m = tof * SPEED_OF_LIGHT`
- `dist_mm = round(dist_m * 1000)`

### Filtri sul tag

Il tag fa già un primo filtro locale prima di mandare i dati a MATLAB.

#### 1. Hard gating sulla distanza

Scarta misure fisicamente improbabili:

- `dist_m < -0.10`
- `dist_m > 20.0`

Se la distanza è solo leggermente negativa per rumore numerico, la forza a `0`.

#### 2. Quality filtering radio

Per ogni `RESP`, il tag calcola una misura di qualità chiamata qui `FP-RX`:

- usa `fp_amp1`, `fp_amp2`, `fp_amp3`
- usa `rx_pream`
- usa `cir_max`

L'idea è:

- se il primo path è coerente con l'energia totale ricevuta, la misura è più probabilmente `LOS`;
- se invece domina il multipath, la misura è più probabilmente `NLOS`.

Le soglie usate sono:

- `FP_RX_DIFF_MIN = -6 dB`
- `MIN_PREAM_COUNT = 80`

Una misura viene marcata `LOS` se:

- `rx_pream >= MIN_PREAM_COUNT`
- `fp_rx_diff >= FP_RX_DIFF_MIN`

altrimenti viene marcata `NLOS`.

#### 3. Deduplica per ancora

Nel round il tag tiene una sola misura per ancora.

Se arrivano due misure della stessa ancora, conserva quella migliore secondo questo ordine:

1. `LOS` batte `NLOS`
2. `QUAL` più alta batte `QUAL` più bassa
3. `pream` più alto batte `pream` più basso

### Output seriale del tag

Per ogni ancora usata nel round stampa una riga:

```text
RANGING MEAS [round] [tag->anchor] distanza_mm QUAL qual_x10 pream FLAG LOS/NLOS
```

Esempio:

```text
RANGING MEAS [211] [54:33->5b:2a] 8276 mm QUAL -41 118 FLAG LOS
```

Alla fine del round stampa anche un riepilogo:

```text
[211] round: 8 meas | 5 LOS | 3 NLOS | 24 timeout | 8 dup
```

Significato dei campi:

- `meas`: numero di ancore uniche conservate nel round
- `LOS`: misure accettate come buone dal filtro radio del tag
- `NLOS`: misure viste ma marcate come scarse
- `timeout`: slot in cui non è arrivata una risposta utile
- `dup`: risposte duplicate della stessa ancora

## File `rng-resp.c`

Questo è il firmware delle `ancore`.

### Cosa fa

- resta in ascolto continuo;
- riceve i pacchetti `INIT`;
- accetta sia `INIT` diretti sia `INIT` broadcast;
- legge subito le diagnostiche del DW1000 dopo la ricezione;
- prepara la `RESP`;
- la trasmette con ritardo programmato nel proprio slot.

### Perché usa gli slot

Tutte le ancore ricevono lo stesso `INIT` broadcast.

Se rispondessero subito tutte insieme, le risposte colliderebbero.

Per evitarlo, ogni ancora usa uno slot diverso definito in `anchor_table.h`.

La funzione chiave è:

- `get_resp_delay_from_table(linkaddr_node_addr)`

che restituisce il ritardo di trasmissione della `RESP`.

### Cosa mette dentro la `RESP`

La `RESP` contiene:

- header IEEE 802.15.4
- `init_rx_ts`
- `resp_tx_ts`
- metriche radio:
  - `fp_amp1`
  - `fp_amp2`
  - `fp_amp3`
  - `rx_pream`
  - `cir_max`
  - `std_noise`

Queste metriche sono lette da:

- `dwt_readdiagnostics(&diag)`

e vengono copiate nel pacchetto prima della trasmissione.

In pratica:

- l'ancora non decide lei la distanza finale;
- l'ancora fornisce al tag i timestamp e gli indicatori di qualità;
- il tag fa la stima finale e il filtraggio.

## Protocollo `INIT -> RESP`

### `INIT`

Il pacchetto `INIT` contiene solo l'header.

Serve per:

- identificare il tag che ha iniziato il round;
- portare il `sequence number` del round;
- dire alle ancore a chi dovranno rispondere.

### `RESP`

Il pacchetto `RESP` viene mandato dall'ancora al tag e contiene:

- i due timestamp necessari al ranging;
- le metriche di qualità del segnale.

Il `sequence number` della `RESP` è lo stesso dell'`INIT`, così il tag capisce a quale round appartiene.

## File `anchor_table.h`

`anchor_table.h` viene generato automaticamente da:

- `generate_and_build_all.py`

e contiene:

- l'elenco delle ancore usate nel tracking;
- lo slot assegnato a ciascuna ancora;
- i parametri temporali del round.

I parametri principali usati dallo script sono:

- `ANCHOR_STEP_UUS = 500`
- `PER_ANCHOR_TIMEOUT_UUS = 700`
- `WINDOW_MARGIN_UUS = 1500`

Quindi il round totale dipende dal numero di ancore attive.

## File `generate_and_build_all.py`

Questo script:

- legge `DEPT_evb1000_map.csv`
- legge `variance_per_node.csv`
- seleziona le ancore richieste per il tracking
- genera `anchor_table.h`
- genera `rng-support.h`
- genera `rng-support.c`
- compila i binari
- copia i `.bin` nella cartella `bins`

Scelta importante del progetto:

- non sovrascrive `rng-init-all.c`
- non sovrascrive `rng-resp.c`

perché questi due file contengono la logica custom del protocollo realtime.

## Cosa flashare

### Tag

Sul tag va flashato:

- `bins/rng-init-all.bin`

Questo tag deve essere collegato via USB al PC, perché la sua seriale è quella letta da MATLAB.

### Ancore

Sulle ancore va flashato:

- `bins/rng-resp.bin`

Le ancore fanno solo da responder e non parlano direttamente con MATLAB.

## Collegamento con MATLAB

`MATLAB` non legge le ancore.

Legge solo la seriale del tag, cioè le righe:

- `RANGING MEAS ...`
- `[round] round: ...`

Quindi la catena completa è:

```text
Tag INIT broadcast
-> Ancore RESP in slot
-> Tag calcola distanza + qualità
-> Tag stampa su seriale
-> MATLAB legge la seriale
```

## Riassunto rapido

- `rng-init-all` = firmware del tag, fa ranging e manda i dati al PC
- `rng-resp` = firmware delle ancore, risponde all'INIT nel proprio slot
- il filtro qualità principale nasce già sul tag
- MATLAB lavora sui dati che il tag ha già aggregato e formattato
