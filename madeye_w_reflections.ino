/*
Hyper-Reactive Eye - La Versione Finale ("Polished Gloss")
Ottimizzato per movimenti fulminei e massima fluidità.
L'effetto di restringimento dei riflessi è stato finemente calibrato per essere
molto più sottile (circa 1/4 della precedente intensità), ottenendo un
effetto lucido finale estremamente realistico e professionale.
*/
#include <Arduino.h>
#include "LCD_Test.h"

extern UWORD *BlackImage;

// Costanti
#define IRIS_BLUR_COLOR 0x021F
#define LUT_SIZE 360

// Variabili Globali
float sin_lut[LUT_SIZE];
float cos_lut[LUT_SIZE];
UWORD Imagesize = LCD_1IN28_HEIGHT * LCD_1IN28_WIDTH * 2;
UWORD *BlackImage;

class HyperEye {
private:
    int _eyeX, _eyeY, _scleraRadius, _irisRadius;
    float _pupilRadius, _pupilTargetRadius;
    float _minPupilRadius, _maxPupilRadius;
    float _irisX, _irisY, _targetX, _targetY;

    // Parametri di movimento
    float _posMoveSpeed = 0.5f;
    float _radiusMoveSpeed = 0.4f;

    // Logica decisionale
    unsigned long _lastDecisionTime = 0;
    unsigned long _actionDuration = 500;

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
            if (decision < 45) { _actionDuration = random(200, 450); _pupilTargetRadius = _minPupilRadius; int angle = random(0, LUT_SIZE); float d = random((_scleraRadius - _irisRadius) * 0.2f, _scleraRadius - _irisRadius); _targetX = _eyeX + cos_lut[angle] * d; _targetY = _eyeY + sin_lut[angle] * d; }
            else if (decision < 75) { _actionDuration = random(800, 2000); _pupilTargetRadius = _maxPupilRadius; }
            else if (decision < 90) { _actionDuration = random(100, 250); int angle = random(0, LUT_SIZE); float d = random(5, 15); _targetX = _irisX + cos_lut[angle] * d; _targetY = _irisY + sin_lut[angle] * d; }
            else { _actionDuration = random(400, 800); _pupilTargetRadius = _maxPupilRadius; int angle = random(0, LUT_SIZE); float d = random((_scleraRadius - _irisRadius) * 0.2f, _scleraRadius - _irisRadius); _targetX = _eyeX + cos_lut[angle] * d; _targetY = _eyeY + sin_lut[angle] * d; }
            _lastDecisionTime = millis();
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
        if (dx_contain * dx_contain + dy_contain * dy_contain > maxDist * maxDist) {
             float dist = sqrt(dx_contain * dx_contain + dy_contain * dy_contain);
             _irisX = _eyeX + (dx_contain / dist) * maxDist;
             _irisY = _eyeY + (dy_contain / dist) * maxDist;
        }

        float dist_from_center = sqrt(dx_contain * dx_contain + dy_contain * dy_contain);
        float perspective_scale = 1.0f - (dist_from_center / maxDist) * 0.15f;
        float irisDrawRadius = _irisRadius * perspective_scale;
        float pupilDrawRadius = _pupilRadius * perspective_scale;

        if (dx_contain * dx_contain + dy_contain * dy_contain > 25.0f) {
            for (int i = 1; i <= 4; i++) {
                Paint_DrawCircle(_irisX - (dx_contain * i * 0.04f), _irisY - (dy_contain * i * 0.04f), irisDrawRadius * (1.0f - (float)i / 5.0f), IRIS_BLUR_COLOR, DOT_PIXEL_1X1, DRAW_FILL_FULL);
            }
        }
        
        Paint_DrawCircle(_irisX, _irisY, irisDrawRadius, BLUE, DOT_PIXEL_1X1, DRAW_FILL_FULL);
        Paint_DrawCircle(_irisX, _irisY, pupilDrawRadius, BLACK, DOT_PIXEL_1X1, DRAW_FILL_FULL);

        float pupilRange = _maxPupilRadius - _minPupilRadius;
        float focusFactor = constrain((_maxPupilRadius - _pupilRadius) / pupilRange, 0.0, 1.0);

        // --- CALIBRAZIONE FINALE DEL RESTRINGIMENTO ---
        // La differenza tra Max e Min è stata ridotta per un effetto più sottile.
        const float posFactorMax = 0.35f, posFactorMin = 0.30f;
        const float sizeFactorMax = 0.25f, sizeFactorMin = 0.19f; // Precedente: 0.10f

        float currentPosFactor = posFactorMax + (posFactorMin - posFactorMax) * focusFactor;
        float currentSizeFactor = sizeFactorMax + (sizeFactorMin - sizeFactorMax) * focusFactor;

        const float posFactor2Max = 0.40f, posFactor2Min = 0.35f;
        const float sizeFactor2Max = 0.15f, sizeFactor2Min = 0.13f; // Precedente: 0.08f
        float currentPosFactor2 = posFactor2Max + (posFactor2Min - posFactor2Max) * focusFactor;
        float currentSizeFactor2 = sizeFactor2Max + (sizeFactor2Min - sizeFactor2Max) * focusFactor;

        int h1X = _irisX + irisDrawRadius * currentPosFactor;
        int h1Y = _irisY - irisDrawRadius * currentPosFactor;
        int h1R = irisDrawRadius * currentSizeFactor;

        int h2X = _irisX - irisDrawRadius * currentPosFactor2;
        int h2Y = _irisY + irisDrawRadius * currentPosFactor2 * 0.5f;
        int h2R = irisDrawRadius * currentSizeFactor2;
        
        Paint_DrawCircle(h1X, h1Y, h1R, WHITE, DOT_PIXEL_1X1, DRAW_FILL_FULL);
        Paint_DrawCircle(h2X, h2Y, h2R, WHITE, DOT_PIXEL_1X1, DRAW_FILL_FULL);
    }
};

HyperEye eye;

void setup() {
    Serial.begin(115200);
    for (int i = 0; i < LUT_SIZE; i++) {
        sin_lut[i] = sin(i * DEG_TO_RAD);
        cos_lut[i] = cos(i * DEG_TO_RAD);
    }
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
    eye.update();
    eye.draw();
    LCD_1IN28_Display(BlackImage);
    delay(0); 
}
