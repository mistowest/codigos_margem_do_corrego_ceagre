#ifndef AJ_SR04M_H
#define AJ_SR04M_H

#include <Arduino.h>

// ============================================================
// CONFIGURACAO - AJUSTE AQUI CONFORME NECESSIDADE
// ============================================================
struct AJ_SR04M_Config {
  // Limites de distancia
  float distanciaMinima = 20.0f;      // Zona morta do sensor (cm)
  float distanciaMaxima = 450.0f;     // Limite maximo do sensor (cm)
  
  // Deteccao de saltos anomalos
  float saltoMaximo = 50.0f;          // Salto maximo permitido entre leituras (cm)
  float velocidadeMaxima = 300.0f;    // Velocidade maxima de variacao (cm/s)
  
  // Comportamento nos limites
  bool clampNaZonaMorta = true;       // true = mantem ultimo valor, false = retorna -1
  bool clampNoMaximo = true;          // true = mantem ultimo valor, false = retorna -1
  
  // Margens de seguranca (evita oscilacao na borda)
  float margemZonaMorta = 5.0f;       // Histerese para zona morta (cm)
  float margemMaximo = 10.0f;         // Histerese para limite maximo (cm)
  
  // Filtros
  uint8_t tamanhoBufMediana = 7;      // Tamanho do buffer para mediana (3-9)
  float alphaEMA = 0.2f;              // Fator de suavizacao EMA (0.1 = lento, 0.5 = rapido)
  uint8_t leiturasParaEstabilizar = 3; // Leituras consecutivas para considerar estavel
  
  // Intervalo
  unsigned long intervaloMs = 80;     // Intervalo entre medicoes (min 60ms)
};

// ============================================================
// CLASSE AJ_SR04M
// ============================================================
class AJ_SR04M {
  public:
    AJ_SR04M(uint8_t trigPin, uint8_t echoPin);
    
    // Inicializacao
    void begin();
    void begin(const AJ_SR04M_Config& config);
    
    // Loop principal
    void update();
    
    // Medicao manual
    bool iniciarMedicao();
    bool leituraDisponivel();
    
    // Leituras
    float getDistancia();           // Ultima leitura valida
    float getDistanciaRaw();        // Leitura bruta (sem filtro)
    float getDistanciaFiltrada();   // Mediana
    float getDistanciaMedia();      // Media movel exponencial
    float getDistanciaEstavel();    // Valor estabilizado
    
    // Configuracao em tempo de execucao
    void setConfig(const AJ_SR04M_Config& config);
    AJ_SR04M_Config getConfig();
    
    // Configuracoes individuais
    void setDistanciaMinima(float cm);
    void setDistanciaMaxima(float cm);
    void setSaltoMaximo(float cm);
    void setVelocidadeMaxima(float cmPorSegundo);
    void setMargemZonaMorta(float cm);
    void setMargemMaximo(float cm);
    void setClampNaZonaMorta(bool ativo);
    void setClampNoMaximo(bool ativo);
    void setIntervalo(unsigned long ms);
    void setModoAutomatico(bool ativo);
    void setAlphaEMA(float alpha);
    
    // Status
    bool sensorOk();
    bool estaEmZonaMorta();
    bool estaNoLimiteMaximo();
    bool estaEstavel();
    uint8_t getErrosConsecutivos();
    unsigned long tempoDesdeUltimaLeitura();
    bool estaMedindo();
    
    // Debug
    void printConfig();
    void printStatus();

  private:
    uint8_t _trigPin;
    uint8_t _echoPin;
    
    enum Estado : uint8_t {
      IDLE,
      TRIGGER_INICIO,
      TRIGGER_ALTO,
      AGUARDANDO_ECHO,
      MEDINDO_ECHO,
      PROCESSANDO
    };
    
    Estado _estado;
    AJ_SR04M_Config _config;
    
    // Temporização
    unsigned long _ultimoUpdateMs;
    unsigned long _tempoTriggerUs;
    unsigned long _tempoEchoInicioUs;
    unsigned long _ultimaLeituraValidaMs;
    unsigned long _tempoUltimaLeituraMs;
    
    bool _modoAutomatico;
    
    // Resultados
    float _distanciaAtual;
    float _distanciaAnterior;
    float _distanciaRaw;
    float _distanciaEstavel;
    bool _novaLeitura;
    uint8_t _errosConsecutivos;
    bool _leituraValida;
    
    // Estados de limite
    bool _emZonaMorta;
    bool _emLimiteMaximo;
    bool _saindoZonaMorta;
    bool _saindoLimiteMaximo;
    
    // Estabilidade
    uint8_t _leiturasEstaveis;
    float _ultimaLeituraEstavel;
    
    // Buffer para mediana
    static const uint8_t MAX_BUFFER = 9;
    float _bufferLeituras[MAX_BUFFER];
    uint8_t _indiceLeitura;
    uint8_t _leiturasNoBuffer;
    
    // Media movel exponencial
    float _mediaMovel;
    bool _mediaInicializada;
    
    // Constantes fisicas
    static constexpr float VELOCIDADE_SOM_CM_US = 0.0343f;
    static const unsigned long TIMEOUT_ECHO_US = 26000;
    static const unsigned long DURACAO_TRIGGER_US = 12;
    static const unsigned long ESTABILIZACAO_US = 4;
    static const uint8_t MAX_ERROS = 10;
    
    // Metodos privados
    void processarLeitura(unsigned long duracaoUs);
    float calcularMediana();
    void adicionarAoBuffer(float valor);
    void resetarFiltros();
    void incrementarErros();
    bool detectarSaltoAnomalo(float novaLeitura);
    bool verificarZonaMorta(float distancia);
    bool verificarLimiteMaximo(float distancia);
    void atualizarEstabilidade(float distancia);
    void aplicarConfig();
};

// ============================================================
// IMPLEMENTACAO
// ============================================================

AJ_SR04M::AJ_SR04M(uint8_t trigPin, uint8_t echoPin) 
  : _trigPin(trigPin)
  , _echoPin(echoPin)
  , _estado(IDLE)
  , _ultimoUpdateMs(0)
  , _tempoTriggerUs(0)
  , _tempoEchoInicioUs(0)
  , _ultimaLeituraValidaMs(0)
  , _tempoUltimaLeituraMs(0)
  , _modoAutomatico(true)
  , _distanciaAtual(0.0f)
  , _distanciaAnterior(-1.0f)
  , _distanciaRaw(0.0f)
  , _distanciaEstavel(0.0f)
  , _novaLeitura(false)
  , _errosConsecutivos(0)
  , _leituraValida(false)
  , _emZonaMorta(false)
  , _emLimiteMaximo(false)
  , _saindoZonaMorta(false)
  , _saindoLimiteMaximo(false)
  , _leiturasEstaveis(0)
  , _ultimaLeituraEstavel(0.0f)
  , _indiceLeitura(0)
  , _leiturasNoBuffer(0)
  , _mediaMovel(0.0f)
  , _mediaInicializada(false)
{
  resetarFiltros();
}

void AJ_SR04M::begin() {
  pinMode(_trigPin, OUTPUT);
  pinMode(_echoPin, INPUT);
  digitalWrite(_trigPin, LOW);
  
  _estado = IDLE;
  _ultimoUpdateMs = millis();
  _errosConsecutivos = 0;
  _distanciaAnterior = -1.0f;
  _emZonaMorta = false;
  _emLimiteMaximo = false;
  
  resetarFiltros();
}

void AJ_SR04M::begin(const AJ_SR04M_Config& config) {
  setConfig(config);
  begin();
}

void AJ_SR04M::setConfig(const AJ_SR04M_Config& config) {
  _config = config;
  aplicarConfig();
}

void AJ_SR04M::aplicarConfig() {
  // Valida limites
  if (_config.distanciaMinima < 0) _config.distanciaMinima = 0;
  if (_config.distanciaMaxima > 450.0f) _config.distanciaMaxima = 450.0f;
  if (_config.distanciaMinima >= _config.distanciaMaxima) {
    _config.distanciaMinima = 20.0f;
    _config.distanciaMaxima = 450.0f;
  }
  
  // Valida intervalo
  if (_config.intervaloMs < 60) _config.intervaloMs = 60;
  
  // Valida buffer
  if (_config.tamanhoBufMediana < 3) _config.tamanhoBufMediana = 3;
  if (_config.tamanhoBufMediana > MAX_BUFFER) _config.tamanhoBufMediana = MAX_BUFFER;
  
  // Valida alpha EMA
  if (_config.alphaEMA < 0.05f) _config.alphaEMA = 0.05f;
  if (_config.alphaEMA > 0.9f) _config.alphaEMA = 0.9f;
}

AJ_SR04M_Config AJ_SR04M::getConfig() {
  return _config;
}

void AJ_SR04M::resetarFiltros() {
  for (uint8_t i = 0; i < MAX_BUFFER; i++) {
    _bufferLeituras[i] = 0.0f;
  }
  _indiceLeitura = 0;
  _leiturasNoBuffer = 0;
  _mediaMovel = 0.0f;
  _mediaInicializada = false;
  _leiturasEstaveis = 0;
}

void AJ_SR04M::incrementarErros() {
  if (_errosConsecutivos < 255) {
    _errosConsecutivos++;
  }
}

// Verifica se esta na zona morta (com histerese)
bool AJ_SR04M::verificarZonaMorta(float distancia) {
  float limiteEntrada = _config.distanciaMinima;
  float limiteSaida = _config.distanciaMinima + _config.margemZonaMorta;
  
  if (_emZonaMorta) {
    // Ja esta na zona morta - precisa sair com margem
    if (distancia > limiteSaida) {
      _saindoZonaMorta = true;
      return false;
    }
    return true;
  } else {
    // Fora da zona morta - verifica entrada
    if (distancia < limiteEntrada) {
      _saindoZonaMorta = false;
      return true;
    }
    return false;
  }
}

// Verifica se esta no limite maximo (com histerese)
bool AJ_SR04M::verificarLimiteMaximo(float distancia) {
  float limiteEntrada = _config.distanciaMaxima;
  float limiteSaida = _config.distanciaMaxima - _config.margemMaximo;
  
  if (_emLimiteMaximo) {
    // Ja esta no limite - precisa sair com margem
    if (distancia < limiteSaida) {
      _saindoLimiteMaximo = true;
      return false;
    }
    return true;
  } else {
    // Fora do limite - verifica entrada
    if (distancia > limiteEntrada) {
      _saindoLimiteMaximo = false;
      return true;
    }
    return false;
  }
}

// Detecta saltos anomalos
bool AJ_SR04M::detectarSaltoAnomalo(float novaLeitura) {
  if (_distanciaAnterior < 0) {
    return false;
  }
  
  unsigned long agoraMs = millis();
  unsigned long deltaMs = agoraMs - _tempoUltimaLeituraMs;
  if (deltaMs == 0) deltaMs = 1;
  
  float diferencaAbs = novaLeitura - _distanciaAnterior;
  if (diferencaAbs < 0) diferencaAbs = -diferencaAbs;
  
  // Verifica salto maximo absoluto
  if (diferencaAbs > _config.saltoMaximo) {
    return true;
  }
  
  // Verifica velocidade maxima
  float velocidadeAtual = (diferencaAbs / deltaMs) * 1000.0f; // cm/s
  if (velocidadeAtual > _config.velocidadeMaxima) {
    return true;
  }
  
  return false;
}

void AJ_SR04M::atualizarEstabilidade(float distancia) {
  float diff = distancia - _ultimaLeituraEstavel;
  if (diff < 0) diff = -diff;
  
  // Se variacao pequena, incrementa contador de estabilidade
  if (diff < 2.0f) {
    if (_leiturasEstaveis < 255) _leiturasEstaveis++;
  } else {
    _leiturasEstaveis = 0;
  }
  
  // Atualiza valor estavel quando estabilizado
  if (_leiturasEstaveis >= _config.leiturasParaEstabilizar) {
    _distanciaEstavel = distancia;
  }
  
  _ultimaLeituraEstavel = distancia;
}

void AJ_SR04M::update() {
  unsigned long agoraMs = millis();
  unsigned long agoraUs = micros();
  
  switch (_estado) {
    case IDLE:
      if (_modoAutomatico && (agoraMs - _ultimoUpdateMs >= _config.intervaloMs)) {
        iniciarMedicao();
      }
      break;
      
    case TRIGGER_INICIO:
      if (agoraUs - _tempoTriggerUs >= ESTABILIZACAO_US) {
        digitalWrite(_trigPin, HIGH);
        _tempoTriggerUs = agoraUs;
        _estado = TRIGGER_ALTO;
      }
      break;
      
    case TRIGGER_ALTO:
      if (agoraUs - _tempoTriggerUs >= DURACAO_TRIGGER_US) {
        digitalWrite(_trigPin, LOW);
        _tempoTriggerUs = agoraUs;
        _estado = AGUARDANDO_ECHO;
      }
      break;
      
    case AGUARDANDO_ECHO:
      if (digitalRead(_echoPin) == HIGH) {
        _tempoEchoInicioUs = agoraUs;
        _estado = MEDINDO_ECHO;
      } 
      else if (agoraUs - _tempoTriggerUs > TIMEOUT_ECHO_US) {
        processarLeitura(0);
        _estado = PROCESSANDO;
      }
      break;
      
    case MEDINDO_ECHO:
      if (digitalRead(_echoPin) == LOW) {
        unsigned long duracaoUs = agoraUs - _tempoEchoInicioUs;
        processarLeitura(duracaoUs);
        _estado = PROCESSANDO;
      }
      else if (agoraUs - _tempoEchoInicioUs > TIMEOUT_ECHO_US) {
        processarLeitura(0);
        _estado = PROCESSANDO;
      }
      break;
      
    case PROCESSANDO:
      _ultimoUpdateMs = agoraMs;
      _estado = IDLE;
      break;
  }
}

bool AJ_SR04M::iniciarMedicao() {
  if (_estado != IDLE) {
    return false;
  }
  
  digitalWrite(_trigPin, LOW);
  _tempoTriggerUs = micros();
  _estado = TRIGGER_INICIO;
  
  return true;
}

void AJ_SR04M::processarLeitura(unsigned long duracaoUs) {
  _novaLeitura = true;
  unsigned long agoraMs = millis();
  
  // Timeout ou sem resposta
  if (duracaoUs == 0) {
    incrementarErros();
    _leituraValida = false;
    return;
  }
  
  // Calcula distancia bruta
  float distancia = (duracaoUs * VELOCIDADE_SOM_CM_US) / 2.0f;
  _distanciaRaw = distancia;
  
  // Verifica zona morta
  bool naZonaMorta = verificarZonaMorta(distancia);
  if (naZonaMorta != _emZonaMorta) {
    _emZonaMorta = naZonaMorta;
  }
  
  if (_emZonaMorta) {
    if (_config.clampNaZonaMorta && _distanciaAnterior > 0) {
      // Mantem ultimo valor valido
      _distanciaAtual = _config.distanciaMinima;
      _leituraValida = true;
    } else {
      _leituraValida = false;
    }
    return;
  }
  
  // Verifica limite maximo
  bool noLimiteMax = verificarLimiteMaximo(distancia);
  if (noLimiteMax != _emLimiteMaximo) {
    _emLimiteMaximo = noLimiteMax;
  }
  
  if (_emLimiteMaximo) {
    if (_config.clampNoMaximo && _distanciaAnterior > 0) {
      // Mantem ultimo valor valido
      _distanciaAtual = _config.distanciaMaxima;
      _leituraValida = true;
    } else {
      _leituraValida = false;
    }
    return;
  }
  
  // Detecta salto anomalo
  if (detectarSaltoAnomalo(distancia)) {
    // Ignora leitura anomala, mantem anterior
    _leituraValida = true;
    return;
  }
  
  // Leitura valida normal
  _distanciaAnterior = _distanciaAtual;
  _distanciaAtual = distancia;
  _errosConsecutivos = 0;
  _leituraValida = true;
  _ultimaLeituraValidaMs = agoraMs;
  _tempoUltimaLeituraMs = agoraMs;
  
  // Atualiza filtros
  adicionarAoBuffer(distancia);
  atualizarEstabilidade(distancia);
  
  // Media movel exponencial
  if (!_mediaInicializada) {
    _mediaMovel = distancia;
    _mediaInicializada = true;
  } else {
    _mediaMovel = (_config.alphaEMA * distancia) + ((1.0f - _config.alphaEMA) * _mediaMovel);
  }
}

void AJ_SR04M::adicionarAoBuffer(float valor) {
  _bufferLeituras[_indiceLeitura] = valor;
  _indiceLeitura = (_indiceLeitura + 1) % _config.tamanhoBufMediana;
  
  if (_leiturasNoBuffer < _config.tamanhoBufMediana) {
    _leiturasNoBuffer++;
  }
}

float AJ_SR04M::calcularMediana() {
  if (_leiturasNoBuffer == 0) {
    return -1.0f;
  }
  
  float temp[MAX_BUFFER];
  for (uint8_t i = 0; i < _leiturasNoBuffer; i++) {
    temp[i] = _bufferLeituras[i];
  }
  
  // Insertion sort
  for (uint8_t i = 1; i < _leiturasNoBuffer; i++) {
    float key = temp[i];
    int8_t j = i - 1;
    while (j >= 0 && temp[j] > key) {
      temp[j + 1] = temp[j];
      j--;
    }
    temp[j + 1] = key;
  }
  
  // Remove outliers se possivel
  uint8_t inicio = 0;
  uint8_t fim = _leiturasNoBuffer;
  
  if (_leiturasNoBuffer >= 5) {
    inicio = 1;
    fim = _leiturasNoBuffer - 1;
  }
  
  // Media dos valores centrais
  float soma = 0;
  uint8_t count = 0;
  for (uint8_t i = inicio; i < fim; i++) {
    soma += temp[i];
    count++;
  }
  
  return (count > 0) ? (soma / count) : temp[_leiturasNoBuffer / 2];
}

// Getters de leitura
bool AJ_SR04M::leituraDisponivel() {
  if (_novaLeitura) {
    _novaLeitura = false;
    return true;
  }
  return false;
}

float AJ_SR04M::getDistancia() {
  return _leituraValida ? _distanciaAtual : -1.0f;
}

float AJ_SR04M::getDistanciaRaw() {
  return _distanciaRaw;
}

float AJ_SR04M::getDistanciaFiltrada() {
  return calcularMediana();
}

float AJ_SR04M::getDistanciaMedia() {
  return _mediaInicializada ? _mediaMovel : -1.0f;
}

float AJ_SR04M::getDistanciaEstavel() {
  return (_leiturasEstaveis >= _config.leiturasParaEstabilizar) ? _distanciaEstavel : _distanciaAtual;
}

// Setters individuais
void AJ_SR04M::setDistanciaMinima(float cm) {
  _config.distanciaMinima = cm;
  aplicarConfig();
}

void AJ_SR04M::setDistanciaMaxima(float cm) {
  _config.distanciaMaxima = cm;
  aplicarConfig();
}

void AJ_SR04M::setSaltoMaximo(float cm) {
  _config.saltoMaximo = cm;
}

void AJ_SR04M::setVelocidadeMaxima(float cmPorSegundo) {
  _config.velocidadeMaxima = cmPorSegundo;
}

void AJ_SR04M::setMargemZonaMorta(float cm) {
  _config.margemZonaMorta = cm;
}

void AJ_SR04M::setMargemMaximo(float cm) {
  _config.margemMaximo = cm;
}

void AJ_SR04M::setClampNaZonaMorta(bool ativo) {
  _config.clampNaZonaMorta = ativo;
}

void AJ_SR04M::setClampNoMaximo(bool ativo) {
  _config.clampNoMaximo = ativo;
}

void AJ_SR04M::setIntervalo(unsigned long ms) {
  _config.intervaloMs = ms;
  aplicarConfig();
}

void AJ_SR04M::setModoAutomatico(bool ativo) {
  _modoAutomatico = ativo;
}

void AJ_SR04M::setAlphaEMA(float alpha) {
  _config.alphaEMA = alpha;
  aplicarConfig();
}

// Status
bool AJ_SR04M::sensorOk() {
  return _errosConsecutivos < MAX_ERROS;
}

bool AJ_SR04M::estaEmZonaMorta() {
  return _emZonaMorta;
}

bool AJ_SR04M::estaNoLimiteMaximo() {
  return _emLimiteMaximo;
}

bool AJ_SR04M::estaEstavel() {
  return _leiturasEstaveis >= _config.leiturasParaEstabilizar;
}

uint8_t AJ_SR04M::getErrosConsecutivos() {
  return _errosConsecutivos;
}

unsigned long AJ_SR04M::tempoDesdeUltimaLeitura() {
  return millis() - _ultimaLeituraValidaMs;
}

bool AJ_SR04M::estaMedindo() {
  return _estado != IDLE;
}

// Debug
void AJ_SR04M::printConfig() {
  Serial.println("=== CONFIG AJ-SR04M ===");
  Serial.print("Dist Min: "); Serial.print(_config.distanciaMinima); Serial.println(" cm");
  Serial.print("Dist Max: "); Serial.print(_config.distanciaMaxima); Serial.println(" cm");
  Serial.print("Salto Max: "); Serial.print(_config.saltoMaximo); Serial.println(" cm");
  Serial.print("Vel Max: "); Serial.print(_config.velocidadeMaxima); Serial.println(" cm/s");
  Serial.print("Margem Min: "); Serial.print(_config.margemZonaMorta); Serial.println(" cm");
  Serial.print("Margem Max: "); Serial.print(_config.margemMaximo); Serial.println(" cm");
  Serial.print("Clamp Min: "); Serial.println(_config.clampNaZonaMorta ? "SIM" : "NAO");
  Serial.print("Clamp Max: "); Serial.println(_config.clampNoMaximo ? "SIM" : "NAO");
  Serial.print("Intervalo: "); Serial.print(_config.intervaloMs); Serial.println(" ms");
  Serial.print("Alpha EMA: "); Serial.println(_config.alphaEMA);
  Serial.println("=======================");
}

void AJ_SR04M::printStatus() {
  Serial.print("Raw: "); Serial.print(_distanciaRaw, 1);
  Serial.print(" | Atual: "); Serial.print(_distanciaAtual, 1);
  Serial.print(" | Filtrada: "); Serial.print(calcularMediana(), 1);
  Serial.print(" | EMA: "); Serial.print(_mediaMovel, 1);
  
  if (_emZonaMorta) Serial.print(" [ZONA MORTA]");
  if (_emLimiteMaximo) Serial.print(" [LIMITE MAX]");
  if (estaEstavel()) Serial.print(" [ESTAVEL]");
  
  Serial.println();
}

#endif // AJ_SR04M_H


// ============================================================
// EXEMPLO DE USO
// ============================================================

#define TRIG_PIN 5
#define ECHO_PIN 18

AJ_SR04M sensor(TRIG_PIN, ECHO_PIN);

unsigned long ultimoLog = 0;

void setup() {
  Serial.begin(115200);
  delay(100);
  
  // OPCAO 1: Configuracao via struct (recomendado)
  AJ_SR04M_Config config;
  config.distanciaMinima = 20.0f;     // Zona morta 20cm
  config.distanciaMaxima = 450.0f;    // Limite maximo 450cm
  config.saltoMaximo = 40.0f;         // Rejeita saltos > 40cm
  config.velocidadeMaxima = 250.0f;   // Max 250 cm/s
  config.margemZonaMorta = 5.0f;      // Histerese 5cm na zona morta
  config.margemMaximo = 10.0f;        // Histerese 10cm no maximo
  config.clampNaZonaMorta = true;     // Mantem 20cm quando entra zona morta
  config.clampNoMaximo = true;        // Mantem 450cm quando excede maximo
  config.intervaloMs = 80;            // Medicao a cada 80ms
  config.alphaEMA = 0.25f;            // Suavizacao media
  config.leiturasParaEstabilizar = 4; // 4 leituras para considerar estavel
  
  sensor.begin(config);
  
  // OPCAO 2: Configuracao individual (alternativa)
  // sensor.begin();
  // sensor.setDistanciaMinima(20.0f);
  // sensor.setDistanciaMaxima(450.0f);
  // sensor.setSaltoMaximo(40.0f);
  // ... etc
  
  sensor.printConfig();
  Serial.println("\nSensor pronto!\n");
}

void loop() {
  sensor.update();
  
  unsigned long agora = millis();
  if (agora - ultimoLog >= 200) {
    ultimoLog = agora;
    
    if (sensor.sensorOk()) {
      sensor.printStatus();
    } else {
      Serial.print("ERRO: ");
      Serial.print(sensor.getErrosConsecutivos());
      Serial.println(" erros consecutivos");
    }
  }
}
