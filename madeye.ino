/*
Hyper-Reactive Eye - La Versione Finale Definitiva (Autonoma)
Ottimizzato per movimenti fulminei e massima fluidità.
Questa versione è stata privata di ogni interazione touch per focalizzarsi
esclusivamente su un comportamento autonomo, instabile e iper-reattivo.
Il codice rappresenta il limite delle performance per questo design.
*/
#include <Arduino.h>
#include "LCD_Test.h"

extern UWORD *BlackImage;

// Costanti
#define IRIS_BLUR_COLOR 0x021F
#define LUT_SIZE 360 // LUT per trigonometria, per massime performance

// Variabili Globali
float sin_lut[LUT_SIZE];
float cos_lut[LUT_SIZE];
UWORD Imagesize = LCD_1IN28_HEIGHT * LCD_1IN28_WIDTH * 2;
UWORD *BlackImage;

class HyperEye {
private:
    // Proprietà geometriche dell'occhio
    int _eyeX, _eyeY;
    int _scleraRadius;
    int _irisRadius;
    float _pupilRadius, _pupilTargetRadius;
    float _minPupilRadius, _maxPupilRadius;

    // Posizione attuale e target dell'iride
    float _irisX, _irisY;
    float _targetX, _targetY;

    // --- OTTIMIZZAZIONE #1: VELOCITÀ FULMINEA ---
    // Valori di "easing" molto alti per movimenti quasi istantanei.
    float _posMoveSpeed = 0.5f;
    float _radiusMoveSpeed = 0.4f;

    // Logica decisionale per il comportamento autonomo
    unsigned long _lastDecisionTime = 0;
    unsigned long _actionDuration = 500; // Durata dell'azione corrente

public:
    void begin() {
        // Inizializzazione delle dimensioni e posizioni
        _eyeX = LCD_1IN28_WIDTH / 2;
        _eyeY = LCD_1IN28_HEIGHT / 2;
        _scleraRadius = LCD_1IN28_WIDTH / 2;
        _irisRadius = _scleraRadius * 0.55f;
        _minPupilRadius = _irisRadius * 0.35f;
        _maxPupilRadius = _irisRadius * 0.65f;

        // Impostazione dello stato iniziale
        _pupilRadius = _pupilTargetRadius = _maxPupilRadius;
        _irisX = _targetX = _eyeX;
        _irisY = _targetY = _eyeY;
    }

    // Metodo di aggiornamento semplificato per il comportamento autonomo
    void update() {
        // Prende una nuova decisione solo dopo che l'azione corrente è terminata
        if (millis() - _lastDecisionTime > _actionDuration) {
            int decision = random(100);

            if (decision < 45) { // 45% probabilità: Saccade + Focus
                int angle_degree = random(0, LUT_SIZE);
                float maxDist = _scleraRadius - _irisRadius;
                float distance = random(maxDist * 0.2f, maxDist);
                _targetX = _eyeX + cos_lut[angle_degree] * distance;
                _targetY = _eyeY + sin_lut[angle_degree] * distance;
                _pupilTargetRadius = _minPupilRadius; // Restringe la pupilla
                
                // --- OTTIMIZZAZIONE #2: AFFINAMENTO PERSONALITÀ ---
                // Movimenti rapidi comportano tempi di focus più brevi
                _actionDuration = random(200, 450);

            } else if (decision < 75) { // 30% probabilità: Unfocus
                _pupilTargetRadius = _maxPupilRadius; // Dilata la pupilla
                _actionDuration = random(800, 2000);

            } else if (decision < 90) { // 15% probabilità: Twitch (movimento nervoso)
                int angle_degree = random(0, LUT_SIZE);
                float distance = random(5, 15);
                _targetX = _irisX + cos_lut[angle_degree] * distance; // Leggero scostamento dalla posizione attuale
                _targetY = _irisY + sin_lut[angle_degree] * distance;
                _actionDuration = random(100, 250);

            } else { // 10% probabilità: Scan (movimento ampio e lento)
                int angle_degree = random(0, LUT_SIZE);
                float maxDist = _scleraRadius - _irisRadius;
                float distance = random(maxDist * 0.2f, maxDist);
                _targetX = _eyeX + cos_lut[angle_degree] * distance;
                _targetY = _eyeY + sin_lut[angle_degree] * distance;
                _pupilTargetRadius = _maxPupilRadius; // Pupilla dilatata
                _actionDuration = random(400, 800);
            }
            
            _lastDecisionTime = millis(); // Resetta il timer della decisione
        }
        
        // Aggiorna fluidamente la posizione dell'iride e il raggio della pupilla
        _irisX += (_targetX - _irisX) * _posMoveSpeed;
        _irisY += (_targetY - _irisY) * _posMoveSpeed;
        _pupilRadius += (_pupilTargetRadius - _pupilRadius) * _radiusMoveSpeed;
    }

    void draw() {
        // Pulisce il buffer
        Paint_Clear(BLACK);

        // Disegna la sclera (parte bianca)
        Paint_DrawCircle(_eyeX, _eyeY, _scleraRadius, WHITE, DOT_PIXEL_1X1, DRAW_FILL_FULL);
        
        // Contenimento dell'iride all'interno della sclera
        float dx_contain = _irisX - _eyeX;
        float dy_contain = _irisY - _eyeY;
        float dist_sq = dx_contain * dx_contain + dy_contain * dy_contain;
        float maxDist = _scleraRadius - _irisRadius;
        if (dist_sq > maxDist * maxDist) {
             float dist = sqrt(dist_sq);
             _irisX = _eyeX + (dx_contain / dist) * maxDist;
             _irisY = _eyeY + (dy_contain / dist) * maxDist;
        }
        
        // Effetto "Motion Blur" per aumentare la sensazione di velocità
        float dx_vel = _targetX - _irisX;
        float dy_vel = _targetY - _irisY;
        float speed_sq = dx_vel * dx_vel + dy_vel * dy_vel;
        if (speed_sq > 25.0f) { // Applica l'effetto solo per movimenti veloci
            int smear_steps = 4;
            for (int i = 1; i <= smear_steps; i++) {
                float smearX = _irisX - (dx_vel * i * 0.04f);
                float smearY = _irisY - (dy_vel * i * 0.04f);
                float smearRadius = _irisRadius * (1.0f - (float)i / (smear_steps + 1));
                Paint_DrawCircle(smearX, smearY, smearRadius, IRIS_BLUR_COLOR, DOT_PIXEL_1X1, DRAW_FILL_FULL);
            }
        }

        // Disegna l'iride e la pupilla
        Paint_DrawCircle(_irisX, _irisY, _irisRadius, BLUE, DOT_PIXEL_1X1, DRAW_FILL_FULL);
        Paint_DrawCircle(_irisX, _irisY, _pupilRadius, BLACK, DOT_PIXEL_1X1, DRAW_FILL_FULL);
    }
};

HyperEye eye;

void setup() {
    Serial.begin(115200);

    // Pre-calcola la Look-Up Table (LUT) per seno e coseno
    for (int i = 0; i < LUT_SIZE; i++) {
        sin_lut[i] = sin(i * DEG_TO_RAD);
        cos_lut[i] = cos(i * DEG_TO_RAD);
    }
    
    delay(100);

    // Inizializza la PSRAM per il buffer dell'immagine
    if (!psramInit()) { 
        Serial.println("PSRAM initialization failed!");
        while(1); 
    }
    if ((BlackImage = (UWORD *)ps_malloc(Imagesize)) == NULL) { 
        Serial.println("PSRAM memory allocation failed!");
        while(1); 
    }

    // Inizializza l'hardware del display
    DEV_Module_Init();
    LCD_1IN28_Init(HORIZONTAL);
    LCD_1IN28_Clear(BLACK);

    // Inizializza il buffer di disegno
    Paint_NewImage((UBYTE *)BlackImage, LCD_1IN28.WIDTH, LCD_1IN28_HEIGHT, 0, BLACK);
    Paint_SetScale(65);

    // Inizializza l'occhio
    eye.begin();

    // Inizializza il generatore di numeri casuali
    randomSeed(analogRead(0));
}

void loop() {
    // Aggiorna lo stato logico dell'occhio
    eye.update();
    
    // Disegna l'occhio nel buffer
    eye.draw();
    
    // Invia il buffer al display per la visualizzazione
    LCD_1IN28_Display(BlackImage);
    
    // Un piccolo delay può aiutare la stabilità su alcuni microcontrollori, 
    // ma 0 è ideale per la massima fluidità.
    delay(0); 
}
