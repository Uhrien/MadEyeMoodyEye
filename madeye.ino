/*
 * Hyper-Reactive Eye - La Versione Finale Definitiva
 *
 * Ottimizzato per movimenti fulminei e massima fluidità.
 * - Velocità di animazione drasticamente aumentata per una reattività "violenta".
 * - Logica comportamentale affinata per una personalità ancora più instabile.
 * - Questo codice rappresenta il limite delle performance per questo design.
*/

#include <Arduino.h>
#include "LCD_Test.h"

extern UWORD *BlackImage;
#define IRIS_BLUR_COLOR 0x021F

// LUT per la trigonometria, il nostro asso nella manica per le performance
#define LUT_SIZE 360
float sin_lut[LUT_SIZE];
float cos_lut[LUT_SIZE];

UWORD Imagesize = LCD_1IN28_HEIGHT * LCD_1IN28_WIDTH * 2;
UWORD *BlackImage; 

CST816S touch(9, 8, 5, 4);

class HyperEye {
private:
    int _eyeX, _eyeY, _scleraRadius, _irisRadius;
    float _pupilRadius, _pupilTargetRadius;
    float _minPupilRadius, _maxPupilRadius;
    float _irisX, _irisY, _targetX, _targetY;
    
    // --- MODIFICA #1: VELOCITÀ FULMINEA ---
    // Aumenta drasticamente la velocità di "easing" per movimenti quasi istantanei.
    float _posMoveSpeed = 0.5f; 
    float _radiusMoveSpeed = 0.4f; 

    // Logica decisionale
    unsigned long _lastDecisionTime = 0;
    unsigned long _actionDuration = 500;
    
public:
    void begin() {
        _eyeX = LCD_1IN28_WIDTH / 2; _eyeY = LCD_1IN28_HEIGHT / 2;
        _scleraRadius = LCD_1IN28_WIDTH / 2;
        _irisRadius = _scleraRadius * 0.55f;
        _minPupilRadius = _irisRadius * 0.35f;
        _maxPupilRadius = _irisRadius * 0.65f;
        
        _pupilRadius = _pupilTargetRadius = _maxPupilRadius;
        _irisX = _targetX = _eyeX; _irisY = _targetY = _eyeY;
    }

    void update(bool isInteracting, int touchX, int touchY) {
        if (isInteracting) {
            _targetX = touchX; _targetY = touchY;
            _pupilTargetRadius = _minPupilRadius;
        } else {
            if (millis() - _lastDecisionTime > _actionDuration) {
                int decision = random(100);

                if (decision < 45) { // Saccade + Focus
                    int angle_degree = random(0, LUT_SIZE);
                    float maxDist = _scleraRadius - _irisRadius;
                    float distance = random(maxDist * 0.2f, maxDist);
                    _targetX = _eyeX + cos_lut[angle_degree] * distance;
                    _targetY = _eyeY + sin_lut[angle_degree] * distance;
                    _pupilTargetRadius = _minPupilRadius;
                    
                    // --- MODIFICA #2: AFFINAMENTO PERSONALITÀ ---
                    // Se si muove più velocemente, tiene il focus per meno tempo.
                    _actionDuration = random(200, 450);   
                
                } else if (decision < 75) { // Unfocus
                    _pupilTargetRadius = _maxPupilRadius;
                    _actionDuration = random(800, 2000);   
                
                } else if (decision < 90) { // Twitch
                    int angle_degree = random(0, LUT_SIZE);
                    float distance = random(5, 15);
                    _targetX = _irisX + cos_lut[angle_degree] * distance;
                    _targetY = _irisY + sin_lut[angle_degree] * distance;
                    _actionDuration = random(100, 250);   
                
                } else { // Scan
                    int angle_degree = random(0, LUT_SIZE);
                    float maxDist = _scleraRadius - _irisRadius;
                    float distance = random(maxDist * 0.2f, maxDist);
                    _targetX = _eyeX + cos_lut[angle_degree] * distance;
                    _targetY = _eyeY + sin_lut[angle_degree] * distance;
                    _pupilTargetRadius = _maxPupilRadius;
                    _actionDuration = random(400, 800);
                }
                
                _lastDecisionTime = millis();
            }
        }
        
        _irisX += (_targetX - _irisX) * _posMoveSpeed;
        _irisY += (_targetY - _irisY) * _posMoveSpeed;
        _pupilRadius += (_pupilTargetRadius - _pupilRadius) * _radiusMoveSpeed;
    }

    void draw() {
        Paint_Clear(BLACK);
        Paint_DrawCircle(_eyeX, _eyeY, _scleraRadius, WHITE, DOT_PIXEL_1X1, DRAW_FILL_FULL);
        
        float dx_contain = _irisX - _eyeX, dy_contain = _irisY - _eyeY;
        float maxDist = _scleraRadius - _irisRadius;
        if (dx_contain*dx_contain > maxDist*maxDist - dy_contain*dy_contain) {
             float dist = sqrt(dx_contain*dx_contain + dy_contain*dy_contain);
             if (dist > maxDist) {
                _irisX = _eyeX + (dx_contain/dist) * maxDist;
                _irisY = _eyeY + (dy_contain/dist) * maxDist;
             }
        }
        
        float dx_vel = _targetX - _irisX, dy_vel = _targetY - _irisY;
        float speed_sq = dx_vel*dx_vel + dy_vel*dy_vel;
        if (speed_sq > 25.0) {
            int smear_steps = 4;
            for (int i = 1; i <= smear_steps; i++) {
                float smearX = _irisX - (dx_vel * i * 0.04f);
                float smearY = _irisY - (dy_vel * i * 0.04f);
                float smearRadius = _irisRadius * (1.0 - (float)i / (smear_steps + 1));
                Paint_DrawCircle(smearX, smearY, smearRadius, IRIS_BLUR_COLOR, DOT_PIXEL_1X1, DRAW_FILL_FULL);
            }
        }
        Paint_DrawCircle(_irisX, _irisY, _irisRadius, BLUE, DOT_PIXEL_1X1, DRAW_FILL_FULL);
        Paint_DrawCircle(_irisX, _irisY, _pupilRadius, BLACK, DOT_PIXEL_1X1, DRAW_FILL_FULL);
    }
};

HyperEye eye;

void setup() {
    Serial.begin(115200);
    for (int i = 0; i < LUT_SIZE; i++) {
        sin_lut[i] = sin(i * DEG_TO_RAD);
        cos_lut[i] = cos(i * DEG_TO_RAD);
    }
    touch.begin();
    delay(100); 
    if (!psramInit()) { while(1); }
    if ((BlackImage = (UWORD *)ps_malloc(Imagesize)) == NULL) { while(1); }
    DEV_Module_Init();
    LCD_1IN28_Init(HORIZONTAL);
    LCD_1IN28_Clear(BLACK);
    Paint_NewImage((UBYTE *)BlackImage, LCD_1IN28.WIDTH, LCD_1IN28_HEIGHT, 0, BLACK);
    Paint_SetScale(65);
    eye.begin();
    randomSeed(analogRead(0));
}

void loop() {
    bool isInteracting = false;
    int touchX = 0, touchY = 0;
    if (touch.available()) {
        if (touch.data.x < LCD_1IN28_WIDTH && touch.data.y < LCD_1IN28_HEIGHT) {
            isInteracting = true;
            touchX = touch.data.x;
            touchY = touch.data.y;
        }
    }
    
    eye.update(isInteracting, touchX, touchY);
    eye.draw();
    LCD_1IN28_Display(BlackImage);
    delay(0); 
}
