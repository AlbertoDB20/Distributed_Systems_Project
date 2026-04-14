# Fase 1: Il Core - Veicolo Singolo Ideale ed EKF

## 1. Obiettivo
Questa fase implementa il fondamento matematico del progetto: la stima dello stato di un singolo veicolo di tipo uniciclo utilizzando un Extended Kalman Filter (EKF). Si ipotizza un ambiente ideale dove tutti i sensori (GPS, Encoder, IMU) operano in modo sincrono alla frequenza fissa di 10 Hz e il segnale GPS è sempre disponibile.

## 2. Definizione del Modello

### 2.1 Vettore di Stato
Lo stato del veicolo è descritto da 5 variabili:
$$x_k = \begin{bmatrix} X_k \\ Y_k \\ \theta_k \\ v_k \\ \omega_k \end{bmatrix}$$
Dove $X_k, Y_k$ sono le coordinate spaziali, $\theta_k$ è l'orientamento, $v_k$ è la velocità lineare scalare e $\omega_k$ la velocità angolare.

### 2.2 Modello di Predizione (Dinamica)
Utilizzando il metodo di Eulero in avanti con tempo di campionamento $T_s$, la transizione di stato è governata da:
$$x_{k+1} = f(x_k) + w_k = \begin{cases} X_k + v_k \cos(\theta_k) T_s \\ Y_k + v_k \sin(\theta_k) T_s \\ \theta_k + \omega_k T_s \\ v_k \\ \omega_k \end{cases} + w_k$$
Lo Jacobiano $A_k = \frac{\partial f}{\partial x}$ calcolato per la linearizzazione locale dell'EKF risulta:
$$A_k = \begin{bmatrix} 1 & 0 & -v_k \sin(\theta_k) T_s & \cos(\theta_k) T_s & 0 \\ 0 & 1 & v_k \cos(\theta_k) T_s & \sin(\theta_k) T_s & 0 \\ 0 & 0 & 1 & 0 & T_s \\ 0 & 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 0 & 1 \end{bmatrix}$$

### 2.3 Modelli di Misura (Sensori)
Le misurazioni $z_k$ dipendono dallo stato $h(x_k)$ con aggiunta di rumore gaussiano $\nu_k \sim \mathcal{N}(0, R)$.
1.  **GPS:** Fornisce le coordinate spaziali assolute.
    $$z_{gps} = \begin{bmatrix} X \\ Y \end{bmatrix} \implies C_{gps} = \begin{bmatrix} 1 & 0 & 0 & 0 & 0 \\ 0 & 1 & 0 & 0 & 0 \end{bmatrix}$$
2.  **IMU:** Fornisce l'orientamento assoluto (magnetometro) e la velocità angolare (giroscopio).
    $$z_{imu} = \begin{bmatrix} \theta \\ \omega \end{bmatrix} \implies C_{imu} = \begin{bmatrix} 0 & 0 & 1 & 0 & 0 \\ 0 & 0 & 0 & 0 & 1 \end{bmatrix}$$
3.  **Encoder:** Misura la velocità angolare delle singole ruote $(\omega_R, \omega_L)$. Sfruttando le relazioni $v = r \frac{\omega_R + \omega_L}{2}$ e $\omega = r \frac{\omega_R - \omega_L}{L}$, si inverte il modello per esprimere le letture in funzione dello stato:
    $$z_{enc} = \begin{bmatrix} \omega_R \\ \omega_L \end{bmatrix} = \begin{bmatrix} \frac{v}{r} + \frac{L \omega}{2r} \\ \frac{v}{r} - \frac{L \omega}{2r} \end{bmatrix} \implies C_{enc} = \begin{bmatrix} 0 & 0 & 0 & \frac{1}{r} & \frac{L}{2r} \\ 0 & 0 & 0 & \frac{1}{r} & -\frac{L}{2r} \end{bmatrix}$$

Le tre matrici $C$ vengono impilate per formare un'unica matrice $C_k$ di dimensione $6 \times 5$.

## 3. Analisi di Osservabilità
In questa configurazione, calcolando la matrice di osservabilità $\mathcal{O} = [C_k; C_k A_k; C_k A_k^2; \dots]$, il rango è pieno (rango 5). Il sistema è completamente e globalmente osservabile grazie alla presenza simultanea del GPS per la posizione assoluta e dell'IMU per l'heading assoluto.