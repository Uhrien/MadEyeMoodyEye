/*
Hyper-Reactive Eye - La Versione Finale Definitiva (Autonoma)
Ottimizzato per movimenti fulminei e massima fluidità.
Questa versione è stata privata di ogni interazione touch per focalizzarsi
esclusivamente su un comportamento autonomo, instabile e iper-reattivo.
Il codice rappresenta il limite delle performance per questo design.

>>> VERSIONE CON ROTAZIONE E MIRRORING CONFIGURABILI <<<
*/
#include <Arduino.h>
#include "LCD_Test.h"

// <<< CONFIGURAZIONE GLOBALE DELL'ORIENTAMENTO >>>
// Modifica queste costanti per cambiare l'orientamento dell'occhio.
//------------------------------------------------------------------
#define ROTATION_DEGREES 90.0f    // Angolo di rotazione in gradi (0, 90, 180, 270, o valori intermedi)
#define MIRROR_HORIZONTAL false  // 'true' per specchiare orizzontalmente (utile per l'occhio sinistro/destro)
#define MIRROR_VERTICAL false    // 'true' per specchiare verticalmente
//------------------------------------------------------------------
// Esempi di configurazione:
// Occhio Destro Standard: ROTATION_DEGREES = 0.0f, MIRROR_HORIZONTAL = false
// Occhio Sinistro (specchiato): ROTATION_DEGREES = 0.0f, MIRROR_HORIZONTAL = true
// Occhio Sottosopra: ROTATION_DEGREES = 180.0f
// Occhio Ruotato a destra: ROTATION_DEGREES = 90.0f

extern UWORD *BlackImage;

// Costanti
#define IRIS_BLUR_COLOR 0x021F
#define LUT_SIZE 360 // LUT per trigonometria
#define HIGHLIGHT_COLOR_AMBIENT 0x8410

// Variabili Globali
float sin_lut[LUT_SIZE];
float cos_lut[LUT_SIZE];
UWORD Imagesize = LCD_1IN28_HEIGHT * LCD_1IN28_WIDTH * 2;
UWORD *BlackImage;

// <<< NUOVE VARIABILI GLOBALI PER LA ROTAZIONE PRE-CALCOLATA >>>
float global_cos_rot;
float global_sin_rot;

// <<< NUOVA FUNZIONE DI UTILITÀ PER LA ROTAZIONE E IL MIRRORING >>>
// Applica la trasformazione globale (rotazione + mirroring) a un vettore (x, y).
// Viene passata per riferimento (&) per massima efficienza.
void transformVector(float& x, float& y) {
    // 1. Applica la rotazione usando la matrice di rotazione standard
    float original_x = x;
    x = original_x * global_cos_rot - y * global_sin_rot;
    y = original_x * global_sin_rot + y * global_cos_rot;

    // 2. Applica il mirroring (dopo la rotazione)
    if (MIRROR_HORIZONTAL) {
        x = -x;
    }
    if (MIRROR_VERTICAL) {
        y = -y;
    }
}


class HyperEye {
private:
    int _eyeX, _eyeY, _scleraRadius, _irisRadius;
    float _pupilRadius, _pupilTargetRadius, _minPupilRadius, _maxPupilRadius;
    float _irisX, _irisY, _targetX, _targetY;
    float _posMoveSpeed = 0.5f, _radiusMoveSpeed = 0.4f;
    float _pupilParallaxFactor = 0.15f; 
    unsigned long _lastDecisionTime = 0, _actionDuration = 500;

public:
    void begin() {
        _eyeX = LCD_1IN28_WIDTH / 2;
        _eyeY = LCD_1IN28_HEIGHT / 2;
        _scleraRadius = LCD_1IN28_WIDTH / 2;
        _irisRadius = _scleraRadius * 0.55f;
        _minPupilRadius = _irisRadius * 0.35f;
        _maxPupilRadius = _irisRadius * 0.65f;
        _pupilRadius = _pupilTargetRadius = _maxPupilRadius;
        _irisX = _targetX = _eyeX;
        _irisY = _targetY = _eyeY;
    }

    void update() {
        if (millis() - _lastDecisionTime > _actionDuration) {
            int decision = random(100);
            int angle_degree;
            float distance;

            if (decision < 45) { // Saccade + Focus
                angle_degree = random(0, LUT_SIZE);
                distance = random((_scleraRadius - _irisRadius) * 0.2f, _scleraRadius - _irisRadius);
                _pupilTargetRadius = _minPupilRadius;
                _actionDuration = random(200, 450);
            } else if (decision < 75) { // Unfocus
                angle_degree = 0; distance = 0; // Rimane fermo
                _pupilTargetRadius = _maxPupilRadius;
                _actionDuration = random(800, 2000);
            } else if (decision < 90) { // Twitch
                angle_degree = random(0, LUT_SIZE);
                distance = random(5, 15);
                _actionDuration = random(100, 250);
            } else { // Scan
                angle_degree = random(0, LUT_SIZE);
                distance = random((_scleraRadius - _irisRadius) * 0.2f, _scleraRadius - _irisRadius);
                _pupilTargetRadius = _maxPupilRadius;
                _actionDuration = random(400, 800);
            }

            // <<< LOGICA DI MOVIMENTO MODIFICATA >>>
            // 1. Calcola il vettore di movimento di base (non ruotato)
            float move_x = cos_lut[angle_degree] * distance;
            float move_y = sin_lut[angle_degree] * distance;
            
            // 2. Ruota e specchia il vettore usando la nostra funzione di utilità
            transformVector(move_x, move_y);
            
            // 3. Applica il vettore trasformato per definire il target
            if (decision < 90 && decision >= 75) { // Se è unfocus o twitch, il target è relativo alla pos attuale
                 _targetX = _irisX + move_x;
                 _targetY = _irisY + move_y;
            } else { // Altrimenti è relativo al centro
                 _targetX = _eyeX + move_x;
                 _targetY = _eyeY + move_y;
            }
            
            _lastDecisionTime = millis();
        }
        
        _irisX += (_targetX - _irisX) * _posMoveSpeed;
        _irisY += (_targetY - _irisY) * _posMoveSpeed;
        _pupilRadius += (_pupilTargetRadius - _pupilRadius) * _radiusMoveSpeed;
    }

    void draw() {
        Paint_Clear(BLACK);
        Paint_DrawCircle(_eyeX, _eyeY, _scleraRadius, WHITE, DOT_PIXEL_1X1, DRAW_FILL_FULL);
        
        // Contenimento iride
        float dx_contain = _irisX - _eyeX, dy_contain = _irisY - _eyeY;
        float dist_sq = dx_contain * dx_contain + dy_contain * dy_contain;
        float maxDist = _scleraRadius - _irisRadius;
        if (dist_sq > maxDist * maxDist) {
             float d = sqrt(dist_sq);
             _irisX = _eyeX + (dx_contain / d) * maxDist;
             _irisY = _eyeY + (dy_contain / d) * maxDist;
        }
        
        // Motion Blur
        float dx_vel = _targetX - _irisX, dy_vel = _targetY - _irisY;
        if (dx_vel * dx_vel + dy_vel * dy_vel > 25.0f) {
            for (int i = 1; i <= 4; i++) {
                Paint_DrawCircle(_irisX - (dx_vel*i*0.04f), _irisY - (dy_vel*i*0.04f), _irisRadius*(1.0f-(float)i/5.0f), IRIS_BLUR_COLOR, DOT_PIXEL_1X1, DRAW_FILL_FULL);
            }
        }

        Paint_DrawCircle(_irisX, _irisY, _irisRadius, BLUE, DOT_PIXEL_1X1, DRAW_FILL_FULL);
        
        // Parallasse Pupilla
        float pupilOffsetX = (_irisX - _eyeX) * _pupilParallaxFactor;
        float pupilOffsetY = (_irisY - _eyeY) * _pupilParallaxFactor;
        Paint_DrawCircle(_irisX + pupilOffsetX, _irisY + pupilOffsetY, _pupilRadius, BLACK, DOT_PIXEL_1X1, DRAW_FILL_FULL);

        // <<< LOGICA RIFLESSI MODIFICATA >>>
        // 1. Definiamo i vettori di offset dei riflessi (non ruotati)
        float highlight1_offset_x = _irisRadius * 0.35f;
        float highlight1_offset_y = -_irisRadius * 0.35f;

        float highlight2_offset_x = -_irisRadius * 0.2f;
        float highlight2_offset_y = _irisRadius * 0.2f;

        // 2. Ruotiamo e specchiamo i vettori
        transformVector(highlight1_offset_x, highlight1_offset_y);
        transformVector(highlight2_offset_x, highlight2_offset_y);

        // 3. Disegniamo i riflessi usando gli offset trasformati
        Paint_DrawCircle(_irisX + highlight1_offset_x, _irisY + highlight1_offset_y, 6, WHITE, DOT_PIXEL_1X1, DRAW_FILL_FULL);
        Paint_DrawCircle(_irisX + highlight2_offset_x, _irisY + highlight2_offset_y, 4, HIGHLIGHT_COLOR_AMBIENT, DOT_PIXEL_1X1, DRAW_FILL_FULL);
    }
};

HyperEye eye;

void setup() {
    Serial.begin(115200);

    // <<< PRE-CALCOLO DELLA ROTAZIONE >>>
    // Calcola il seno e il coseno dell'angolo di rotazione UNA SOLA VOLTA.
    float rotation_rad = ROTATION_DEGREES * DEG_TO_RAD;
    global_cos_rot = cos(rotation_rad);
    global_sin_rot = sin(rotation_rad);

    for (int i = 0; i < LUT_SIZE; i++) {
        sin_lut[i] = sin(i * DEG_TO_RAD);
        cos_lut[i] = cos(i * DEG_TO_RAD);
    }
    
    delay(100);

    if (!psramInit()) { while(1); }
    if ((BlackImage = (UWORD *)ps_malloc(Imagesize)) == NULL) { while(1); }

    DEV_Module_Init();
    // Imposta l'orientamento fisico del display. HORIZONTAL è un buon default.
    // La rotazione logica verrà gestita dal nostro codice.
    LCD_1IN28_Init(HORIZONTAL);
    LCD_1IN28_Clear(BLACK);
    
    Paint_NewImage((UBYTE *)BlackImage, LCD_1IN28.WIDTH, LCD_1IN28_HEIGHT, 0, BLACK);
    Paint_SetScale(65);

    eye.begin();
    randomSeed(analogRead(0));
}

void loop() {
    eye.update();
    eye.draw();
    LCD_1IN28_Display(BlackImage);
    delay(0); 
}
