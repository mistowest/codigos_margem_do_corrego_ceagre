#ifndef SHT3X_H
#define SHT3X_H

#include <Arduino.h>
#include <Wire.h>

// ============================================================
// CONFIGURACAO SHT3X
// ============================================================
struct SHT3X_Config {
  // Endereco I2C (0x44 ou 0x45)
  uint8_t endereco = 0x44;
  
  // Modo de medicao
  // 0 = Single Shot High Rep
  // 1 = Single Shot Medium Rep
  // 2 = Single Shot Low Rep
  uint8_t modoMedicao = 0;
  
  // Intervalo entre medicoes (ms)
  unsigned long intervaloMs = 500;
  
  // Filtros
  uint8_t tamanhoBufMediana = 5;      // 3-9
  float alphaEMA = 0.3f;              // 0.1 = lento, 0.5 = rapido
  
  // Limites de alerta
  float tempMinAlerta = -40.0f;
  float tempMaxAlerta = 125.0f;
  float umidMinAlerta = 0.0f;
  float umidMaxAlerta = 100.0f;
  
  // Compensacao
  float offsetTemperatura = 0.0f;     // Offset de calibracao (C)
  float offsetUmidade = 0.0f;         // Offset de calibracao (%)
  
  // Validacao
  float variacaoMaxTemp = 10.0f;      // Variacao maxima entre leituras (C)
  float variacaoMaxUmid = 20.0f;      // Variacao maxima entre leituras (%)
  
  // Aquecedor interno
  bool aquecedorAtivo = false;
};

// ============================================================
// ESTRUTURA DE DADOS
// ============================================================
struct SHT3X_Dados {
  float temperatura;        // Celsius
  float umidade;           // % RH
  float pontoOrvalho;      // Celsius
  float indiceCalor;       // Celsius (sensacao termica)
  float umidadeAbsoluta;   // g/m3
  float pressaoVapor;      // hPa
  bool valido;
};

// ============================================================
// CLASSE SHT3X
// ============================================================
class SHT3X {
  public:
    SHT3X(TwoWire* wire = &Wire);
    
    // Inicializacao
    bool begin(uint8_t sda = 21, uint8_t scl = 22, uint32_t frequencia = 100000);
    bool begin(const SHT3X_Config& config, uint8_t sda = 21, uint8_t scl = 22);
    
    // Loop principal
    void update();
    
    // Medicao manual
    bool iniciarMedicao();
    bool leituraDisponivel();
    
    // Leituras de temperatura
    float getTemperatura();
    float getTemperaturaRaw();
    float getTemperaturaFiltrada();
    float getTemperaturaMedia();
    float getTemperaturaEstavel();
    float getTemperaturaFahrenheit();
    float getTemperaturaKelvin();
    
    // Leituras de umidade
    float getUmidade();
    float getUmidadeRaw();
    float getUmidadeFiltrada();
    float getUmidadeMedia();
    float getUmidadeEstavel();
    
    // Calculos derivados
    float getPontoOrvalho();
    float getIndiceCalor();
    float getUmidadeAbsoluta();
    float getPressaoVapor();
    float getUmidadeEspecifica(float pressaoAtm = 1013.25f);
    
    // Dados completos
    SHT3X_Dados getDados();
    
    // Configuracao
    void setConfig(const SHT3X_Config& config);
    SHT3X_Config getConfig();
    
    // Configuracoes individuais
    void setIntervalo(unsigned long ms);
    void setModoMedicao(uint8_t modo);
    void setOffsetTemperatura(float offset);
    void setOffsetUmidade(float offset);
    void setLimitesAlertaTemp(float min, float max);
    void setLimitesAlertaUmid(float min, float max);
    void setModoAutomatico(bool ativo);
    void setAlphaEMA(float alpha);
    
    // Aquecedor interno
    bool ativarAquecedor(bool ativo);
    bool aquecedorAtivo();
    
    // Comandos
    bool softReset();
    bool clearStatus();
    uint16_t lerStatusRegister();
    
    // Status
    bool sensorOk();
    bool sensorConectado();
    bool alertaTemperatura();
    bool alertaUmidade();
    bool estaEstavel();
    uint8_t getErrosConsecutivos();
    unsigned long tempoDesdeUltimaLeitura();
    bool estaMedindo();
    
    // Numero de serie
    uint32_t getNumeroSerie();
    
    // Debug
    void printConfig();
    void printStatus();
    void printDados();

  private:
    TwoWire* _wire;
    SHT3X_Config _config;
    
    // Estados
    enum Estado : uint8_t {
      IDLE,
      ENVIANDO_COMANDO,
      AGUARDANDO_MEDICAO,
      LENDO_DADOS,
      PROCESSANDO
    };
    
    Estado _estado;
    
    // Temporização
    unsigned long _ultimoUpdateMs;
    unsigned long _tempoComandoMs;
    unsigned long _ultimaLeituraValidaMs;
    unsigned long _tempoEsperaMedicao;
    
    bool _modoAutomatico;
    bool _sensorPresente;
    bool _aquecedorLigado;
    
    // Resultados
    float _temperaturaAtual;
    float _temperaturaAnterior;
    float _temperaturaRaw;
    float _umidadeAtual;
    float _umidadeAnterior;
    float _umidadeRaw;
    
    bool _novaLeitura;
    uint8_t _errosConsecutivos;
    bool _leituraValida;
    
    // Alertas
    bool _alertaTemp;
    bool _alertaUmid;
    
    // Estabilidade
    uint8_t _leiturasEstaveis;
    float _tempEstavel;
    float _umidEstavel;
    
    // Buffers para filtros
    static const uint8_t MAX_BUFFER = 9;
    float _bufferTemp[MAX_BUFFER];
    float _bufferUmid[MAX_BUFFER];
    uint8_t _indiceLeitura;
    uint8_t _leiturasNoBuffer;
    
    // Media movel exponencial
    float _mediaTempEMA;
    float _mediaUmidEMA;
    bool _emaInicializada;
    
    // Buffer I2C
    uint8_t _bufferI2C[6];
    
    // Constantes
    static const uint8_t MAX_ERROS = 10;
    
    // Comandos SHT3x
    static const uint16_t CMD_MEAS_HIGH_REP    = 0x2400;
    static const uint16_t CMD_MEAS_MED_REP     = 0x240B;
    static const uint16_t CMD_MEAS_LOW_REP     = 0x2416;
    static const uint16_t CMD_HEATER_ENABLE    = 0x306D;
    static const uint16_t CMD_HEATER_DISABLE   = 0x3066;
    static const uint16_t CMD_SOFT_RESET       = 0x30A2;
    static const uint16_t CMD_READ_STATUS      = 0xF32D;
    static const uint16_t CMD_CLEAR_STATUS     = 0x3041;
    static const uint16_t CMD_READ_SERIAL_H    = 0x3780;
    static const uint16_t CMD_READ_SERIAL_L    = 0x3682;
    
    // Tempos de espera por modo (ms)
    static const uint8_t TEMPO_HIGH_REP = 16;
    static const uint8_t TEMPO_MED_REP  = 7;
    static const uint8_t TEMPO_LOW_REP  = 5;
    
    // Metodos privados
    bool enviarComando(uint16_t comando);
    bool lerDados(uint8_t* buffer, uint8_t tamanho);
    uint8_t calcularCRC(uint8_t* dados, uint8_t tamanho);
    bool verificarCRC(uint8_t* dados, uint8_t crcRecebido);
    void processarLeitura();
    float calcularTemperatura(uint16_t rawTemp);
    float calcularUmidade(uint16_t rawUmid);
    float calcularMediana(float* buffer);
    void adicionarAoBuffer(float temp, float umid);
    void resetarFiltros();
    void incrementarErros();
    bool validarLeitura(float temp, float umid);
    void atualizarEstabilidade(float temp, float umid);
    void verificarAlertas();
    void aplicarConfig();
    uint16_t getComandoMedicao();
    uint8_t getTempoMedicao();
    
    // Calculos
    float calcularPontoOrvalho(float temp, float umid);
    float calcularIndiceCalor(float temp, float umid);
    float calcularUmidadeAbsoluta(float temp, float umid);
    float calcularPressaoVapor(float temp, float umid);
};

// ============================================================
// IMPLEMENTACAO
// ============================================================

SHT3X::SHT3X(TwoWire* wire)
  : _wire(wire)
  , _estado(IDLE)
  , _ultimoUpdateMs(0)
  , _tempoComandoMs(0)
  , _ultimaLeituraValidaMs(0)
  , _tempoEsperaMedicao(TEMPO_HIGH_REP)
  , _modoAutomatico(true)
  , _sensorPresente(false)
  , _aquecedorLigado(false)
  , _temperaturaAtual(0.0f)
  , _temperaturaAnterior(-999.0f)
  , _temperaturaRaw(0.0f)
  , _umidadeAtual(0.0f)
  , _umidadeAnterior(-999.0f)
  , _umidadeRaw(0.0f)
  , _novaLeitura(false)
  , _errosConsecutivos(0)
  , _leituraValida(false)
  , _alertaTemp(false)
  , _alertaUmid(false)
  , _leiturasEstaveis(0)
  , _tempEstavel(0.0f)
  , _umidEstavel(0.0f)
  , _indiceLeitura(0)
  , _leiturasNoBuffer(0)
  , _mediaTempEMA(0.0f)
  , _mediaUmidEMA(0.0f)
  , _emaInicializada(false)
{
  resetarFiltros();
}

bool SHT3X::begin(uint8_t sda, uint8_t scl, uint32_t frequencia) {
  _wire->begin(sda, scl);
  _wire->setClock(frequencia);
  
  delay(1);
  
  // Tenta soft reset
  if (!softReset()) {
    _sensorPresente = false;
    return false;
  }
  
  delay(2);
  
  // Verifica comunicacao
  _wire->beginTransmission(_config.endereco);
  if (_wire->endTransmission() != 0) {
    _sensorPresente = false;
    return false;
  }
  
  _sensorPresente = true;
  _estado = IDLE;
  _ultimoUpdateMs = millis();
  
  aplicarConfig();
  resetarFiltros();
  
  return true;
}

bool SHT3X::begin(const SHT3X_Config& config, uint8_t sda, uint8_t scl) {
  _config = config;
  aplicarConfig();
  return begin(sda, scl);
}

void SHT3X::aplicarConfig() {
  // Valida intervalo
  if (_config.intervaloMs < 100) _config.intervaloMs = 100;
  
  // Valida buffer
  if (_config.tamanhoBufMediana < 3) _config.tamanhoBufMediana = 3;
  if (_config.tamanhoBufMediana > MAX_BUFFER) _config.tamanhoBufMediana = MAX_BUFFER;
  
  // Valida alpha
  if (_config.alphaEMA < 0.05f) _config.alphaEMA = 0.05f;
  if (_config.alphaEMA > 0.9f) _config.alphaEMA = 0.9f;
  
  // Valida modo
  if (_config.modoMedicao > 2) _config.modoMedicao = 0;
  
  // Atualiza tempo de espera
  _tempoEsperaMedicao = getTempoMedicao();
  
  // Configura aquecedor
  if (_sensorPresente) {
    ativarAquecedor(_config.aquecedorAtivo);
  }
}

void SHT3X::setConfig(const SHT3X_Config& config) {
  _config = config;
  aplicarConfig();
}

SHT3X_Config SHT3X::getConfig() {
  return _config;
}

void SHT3X::resetarFiltros() {
  for (uint8_t i = 0; i < MAX_BUFFER; i++) {
    _bufferTemp[i] = 0.0f;
    _bufferUmid[i] = 0.0f;
  }
  _indiceLeitura = 0;
  _leiturasNoBuffer = 0;
  _mediaTempEMA = 0.0f;
  _mediaUmidEMA = 0.0f;
  _emaInicializada = false;
  _leiturasEstaveis = 0;
}

void SHT3X::incrementarErros() {
  if (_errosConsecutivos < 255) {
    _errosConsecutivos++;
  }
}

uint16_t SHT3X::getComandoMedicao() {
  switch (_config.modoMedicao) {
    case 1:  return CMD_MEAS_MED_REP;
    case 2:  return CMD_MEAS_LOW_REP;
    default: return CMD_MEAS_HIGH_REP;
  }
}

uint8_t SHT3X::getTempoMedicao() {
  switch (_config.modoMedicao) {
    case 1:  return TEMPO_MED_REP;
    case 2:  return TEMPO_LOW_REP;
    default: return TEMPO_HIGH_REP;
  }
}

bool SHT3X::enviarComando(uint16_t comando) {
  _wire->beginTransmission(_config.endereco);
  _wire->write(comando >> 8);
  _wire->write(comando & 0xFF);
  return (_wire->endTransmission() == 0);
}

bool SHT3X::lerDados(uint8_t* buffer, uint8_t tamanho) {
  uint8_t recebidos = _wire->requestFrom(_config.endereco, tamanho);
  if (recebidos != tamanho) {
    return false;
  }
  
  for (uint8_t i = 0; i < tamanho; i++) {
    buffer[i] = _wire->read();
  }
  
  return true;
}

uint8_t SHT3X::calcularCRC(uint8_t* dados, uint8_t tamanho) {
  uint8_t crc = 0xFF;
  
  for (uint8_t i = 0; i < tamanho; i++) {
    crc ^= dados[i];
    for (uint8_t bit = 0; bit < 8; bit++) {
      if (crc & 0x80) {
        crc = (crc << 1) ^ 0x31;
      } else {
        crc <<= 1;
      }
    }
  }
  
  return crc;
}

bool SHT3X::verificarCRC(uint8_t* dados, uint8_t crcRecebido) {
  return (calcularCRC(dados, 2) == crcRecebido);
}

float SHT3X::calcularTemperatura(uint16_t rawTemp) {
  return -45.0f + (175.0f * rawTemp / 65535.0f);
}

float SHT3X::calcularUmidade(uint16_t rawUmid) {
  float umid = 100.0f * rawUmid / 65535.0f;
  if (umid < 0.0f) umid = 0.0f;
  if (umid > 100.0f) umid = 100.0f;
  return umid;
}

void SHT3X::update() {
  if (!_sensorPresente) return;
  
  unsigned long agoraMs = millis();
  
  switch (_estado) {
    case IDLE:
      if (_modoAutomatico && (agoraMs - _ultimoUpdateMs >= _config.intervaloMs)) {
        iniciarMedicao();
      }
      break;
      
    case ENVIANDO_COMANDO:
      if (enviarComando(getComandoMedicao())) {
        _tempoComandoMs = agoraMs;
        _estado = AGUARDANDO_MEDICAO;
      } else {
        incrementarErros();
        _estado = IDLE;
        _ultimoUpdateMs = agoraMs;
      }
      break;
      
    case AGUARDANDO_MEDICAO:
      if (agoraMs - _tempoComandoMs >= _tempoEsperaMedicao) {
        _estado = LENDO_DADOS;
      }
      break;
      
    case LENDO_DADOS:
      if (lerDados(_bufferI2C, 6)) {
        _estado = PROCESSANDO;
        processarLeitura();
      } else {
        incrementarErros();
        _leituraValida = false;
        _estado = IDLE;
        _ultimoUpdateMs = agoraMs;
      }
      break;
      
    case PROCESSANDO:
      _ultimoUpdateMs = agoraMs;
      _estado = IDLE;
      break;
  }
}

bool SHT3X::iniciarMedicao() {
  if (_estado != IDLE || !_sensorPresente) {
    return false;
  }
  
  _estado = ENVIANDO_COMANDO;
  return true;
}

void SHT3X::processarLeitura() {
  _novaLeitura = true;
  
  // Verifica CRC temperatura
  if (!verificarCRC(_bufferI2C, _bufferI2C[2])) {
    incrementarErros();
    _leituraValida = false;
    return;
  }
  
  // Verifica CRC umidade
  if (!verificarCRC(_bufferI2C + 3, _bufferI2C[5])) {
    incrementarErros();
    _leituraValida = false;
    return;
  }
  
  // Extrai valores raw
  uint16_t rawTemp = ((uint16_t)_bufferI2C[0] << 8) | _bufferI2C[1];
  uint16_t rawUmid = ((uint16_t)_bufferI2C[3] << 8) | _bufferI2C[4];
  
  // Calcula valores
  float temp = calcularTemperatura(rawTemp) + _config.offsetTemperatura;
  float umid = calcularUmidade(rawUmid) + _config.offsetUmidade;
  
  // Limita umidade
  if (umid < 0.0f) umid = 0.0f;
  if (umid > 100.0f) umid = 100.0f;
  
  _temperaturaRaw = temp;
  _umidadeRaw = umid;
  
  // Valida leitura
  if (!validarLeitura(temp, umid)) {
    incrementarErros();
    _leituraValida = false;
    return;
  }
  
  // Leitura valida
  _temperaturaAnterior = _temperaturaAtual;
  _umidadeAnterior = _umidadeAtual;
  _temperaturaAtual = temp;
  _umidadeAtual = umid;
  _errosConsecutivos = 0;
  _leituraValida = true;
  _ultimaLeituraValidaMs = millis();
  
  // Atualiza filtros
  adicionarAoBuffer(temp, umid);
  atualizarEstabilidade(temp, umid);
  
  // EMA
  if (!_emaInicializada) {
    _mediaTempEMA = temp;
    _mediaUmidEMA = umid;
    _emaInicializada = true;
  } else {
    _mediaTempEMA = (_config.alphaEMA * temp) + ((1.0f - _config.alphaEMA) * _mediaTempEMA);
    _mediaUmidEMA = (_config.alphaEMA * umid) + ((1.0f - _config.alphaEMA) * _mediaUmidEMA);
  }
  
  // Verifica alertas
  verificarAlertas();
}

bool SHT3X::validarLeitura(float temp, float umid) {
  // Primeira leitura sempre valida
  if (_temperaturaAnterior < -900.0f) {
    return true;
  }
  
  // Verifica variacao de temperatura
  float diffTemp = temp - _temperaturaAnterior;
  if (diffTemp < 0) diffTemp = -diffTemp;
  if (diffTemp > _config.variacaoMaxTemp) {
    return false;
  }
  
  // Verifica variacao de umidade
  float diffUmid = umid - _umidadeAnterior;
  if (diffUmid < 0) diffUmid = -diffUmid;
  if (diffUmid > _config.variacaoMaxUmid) {
    return false;
  }
  
  return true;
}

void SHT3X::atualizarEstabilidade(float temp, float umid) {
  float diffTemp = temp - _tempEstavel;
  float diffUmid = umid - _umidEstavel;
  if (diffTemp < 0) diffTemp = -diffTemp;
  if (diffUmid < 0) diffUmid = -diffUmid;
  
  if (diffTemp < 0.5f && diffUmid < 1.0f) {
    if (_leiturasEstaveis < 255) _leiturasEstaveis++;
  } else {
    _leiturasEstaveis = 0;
  }
  
  if (_leiturasEstaveis >= 3) {
    _tempEstavel = temp;
    _umidEstavel = umid;
  } else {
    _tempEstavel = temp;
    _umidEstavel = umid;
  }
}

void SHT3X::verificarAlertas() {
  _alertaTemp = (_temperaturaAtual < _config.tempMinAlerta || 
                 _temperaturaAtual > _config.tempMaxAlerta);
  _alertaUmid = (_umidadeAtual < _config.umidMinAlerta || 
                 _umidadeAtual > _config.umidMaxAlerta);
}

void SHT3X::adicionarAoBuffer(float temp, float umid) {
  _bufferTemp[_indiceLeitura] = temp;
  _bufferUmid[_indiceLeitura] = umid;
  _indiceLeitura = (_indiceLeitura + 1) % _config.tamanhoBufMediana;
  
  if (_leiturasNoBuffer < _config.tamanhoBufMediana) {
    _leiturasNoBuffer++;
  }
}

float SHT3X::calcularMediana(float* buffer) {
  if (_leiturasNoBuffer == 0) {
    return -999.0f;
  }
  
  float temp[MAX_BUFFER];
  for (uint8_t i = 0; i < _leiturasNoBuffer; i++) {
    temp[i] = buffer[i];
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
  
  // Mediana
  uint8_t meio = _leiturasNoBuffer / 2;
  if (_leiturasNoBuffer % 2 == 0) {
    return (temp[meio - 1] + temp[meio]) / 2.0f;
  }
  return temp[meio];
}

// ============================================================
// CALCULOS DERIVADOS
// ============================================================

float SHT3X::calcularPontoOrvalho(float temp, float umid) {
  // Formula Magnus-Tetens
  const float a = 17.27f;
  const float b = 237.7f;
  
  float gamma = (a * temp / (b + temp)) + log(umid / 100.0f);
  return (b * gamma) / (a - gamma);
}

float SHT3X::calcularIndiceCalor(float temp, float umid) {
  // Formula simplificada do NWS (valida para T > 26C e RH > 40%)
  if (temp < 26.0f) return temp;
  
  float T = temp;
  float R = umid;
  
  float HI = -8.78469475556f 
           + 1.61139411f * T 
           + 2.33854883889f * R
           - 0.14611605f * T * R
           - 0.012308094f * T * T
           - 0.0164248277778f * R * R
           + 0.002211732f * T * T * R
           + 0.00072546f * T * R * R
           - 0.000003582f * T * T * R * R;
  
  return HI;
}

float SHT3X::calcularUmidadeAbsoluta(float temp, float umid) {
  // g/m3
  float Psat = 6.112f * exp((17.67f * temp) / (temp + 243.5f));
  float Pv = (umid / 100.0f) * Psat;
  return (Pv * 100.0f) / (461.5f * (temp + 273.15f)) * 1000.0f;
}

float SHT3X::calcularPressaoVapor(float temp, float umid) {
  // hPa
  float Psat = 6.112f * exp((17.67f * temp) / (temp + 243.5f));
  return (umid / 100.0f) * Psat;
}

// ============================================================
// GETTERS TEMPERATURA
// ============================================================

bool SHT3X::leituraDisponivel() {
  if (_novaLeitura) {
    _novaLeitura = false;
    return true;
  }
  return false;
}

float SHT3X::getTemperatura() {
  return _leituraValida ? _temperaturaAtual : -999.0f;
}

float SHT3X::getTemperaturaRaw() {
  return _temperaturaRaw;
}

float SHT3X::getTemperaturaFiltrada() {
  return calcularMediana(_bufferTemp);
}

float SHT3X::getTemperaturaMedia() {
  return _emaInicializada ? _mediaTempEMA : -999.0f;
}

float SHT3X::getTemperaturaEstavel() {
  return (_leiturasEstaveis >= 3) ? _tempEstavel : _temperaturaAtual;
}

float SHT3X::getTemperaturaFahrenheit() {
  return (_temperaturaAtual * 9.0f / 5.0f) + 32.0f;
}

float SHT3X::getTemperaturaKelvin() {
  return _temperaturaAtual + 273.15f;
}

// ============================================================
// GETTERS UMIDADE
// ============================================================

float SHT3X::getUmidade() {
  return _leituraValida ? _umidadeAtual : -999.0f;
}

float SHT3X::getUmidadeRaw() {
  return _umidadeRaw;
}

float SHT3X::getUmidadeFiltrada() {
  return calcularMediana(_bufferUmid);
}

float SHT3X::getUmidadeMedia() {
  return _emaInicializada ? _mediaUmidEMA : -999.0f;
}

float SHT3X::getUmidadeEstavel() {
  return (_leiturasEstaveis >= 3) ? _umidEstavel : _umidadeAtual;
}

// ============================================================
// GETTERS DERIVADOS
// ============================================================

float SHT3X::getPontoOrvalho() {
  if (!_leituraValida) return -999.0f;
  return calcularPontoOrvalho(_temperaturaAtual, _umidadeAtual);
}

float SHT3X::getIndiceCalor() {
  if (!_leituraValida) return -999.0f;
  return calcularIndiceCalor(_temperaturaAtual, _umidadeAtual);
}

float SHT3X::getUmidadeAbsoluta() {
  if (!_leituraValida) return -999.0f;
  return calcularUmidadeAbsoluta(_temperaturaAtual, _umidadeAtual);
}

float SHT3X::getPressaoVapor() {
  if (!_leituraValida) return -999.0f;
  return calcularPressaoVapor(_temperaturaAtual, _umidadeAtual);
}

float SHT3X::getUmidadeEspecifica(float pressaoAtm) {
  if (!_leituraValida) return -999.0f;
  float Pv = calcularPressaoVapor(_temperaturaAtual, _umidadeAtual);
  return 0.622f * Pv / (pressaoAtm - Pv) * 1000.0f; // g/kg
}

SHT3X_Dados SHT3X::getDados() {
  SHT3X_Dados dados;
  dados.temperatura = _temperaturaAtual;
  dados.umidade = _umidadeAtual;
  dados.pontoOrvalho = calcularPontoOrvalho(_temperaturaAtual, _umidadeAtual);
  dados.indiceCalor = calcularIndiceCalor(_temperaturaAtual, _umidadeAtual);
  dados.umidadeAbsoluta = calcularUmidadeAbsoluta(_temperaturaAtual, _umidadeAtual);
  dados.pressaoVapor = calcularPressaoVapor(_temperaturaAtual, _umidadeAtual);
  dados.valido = _leituraValida;
  return dados;
}

// ============================================================
// SETTERS INDIVIDUAIS
// ============================================================

void SHT3X::setIntervalo(unsigned long ms) {
  _config.intervaloMs = ms;
  aplicarConfig();
}

void SHT3X::setModoMedicao(uint8_t modo) {
  _config.modoMedicao = modo;
  aplicarConfig();
}

void SHT3X::setOffsetTemperatura(float offset) {
  _config.offsetTemperatura = offset;
}

void SHT3X::setOffsetUmidade(float offset) {
  _config.offsetUmidade = offset;
}

void SHT3X::setLimitesAlertaTemp(float min, float max) {
  _config.tempMinAlerta = min;
  _config.tempMaxAlerta = max;
}

void SHT3X::setLimitesAlertaUmid(float min, float max) {
  _config.umidMinAlerta = min;
  _config.umidMaxAlerta = max;
}

void SHT3X::setModoAutomatico(bool ativo) {
  _modoAutomatico = ativo;
}

void SHT3X::setAlphaEMA(float alpha) {
  _config.alphaEMA = alpha;
  aplicarConfig();
}

// ============================================================
// COMANDOS
// ============================================================

bool SHT3X::ativarAquecedor(bool ativo) {
  if (!_sensorPresente) return false;
  
  uint16_t cmd = ativo ? CMD_HEATER_ENABLE : CMD_HEATER_DISABLE;
  if (enviarComando(cmd)) {
    _aquecedorLigado = ativo;
    return true;
  }
  return false;
}

bool SHT3X::aquecedorAtivo() {
  return _aquecedorLigado;
}

bool SHT3X::softReset() {
  return enviarComando(CMD_SOFT_RESET);
}

bool SHT3X::clearStatus() {
  return enviarComando(CMD_CLEAR_STATUS);
}

uint16_t SHT3X::lerStatusRegister() {
  if (!enviarComando(CMD_READ_STATUS)) {
    return 0xFFFF;
  }
  
  delay(1);
  
  uint8_t buffer[3];
  if (!lerDados(buffer, 3)) {
    return 0xFFFF;
  }
  
  if (!verificarCRC(buffer, buffer[2])) {
    return 0xFFFF;
  }
  
  return ((uint16_t)buffer[0] << 8) | buffer[1];
}

uint32_t SHT3X::getNumeroSerie() {
  if (!_sensorPresente) return 0;
  
  uint32_t serial = 0;
  uint8_t buffer[6];
  
  // Parte alta
  if (enviarComando(CMD_READ_SERIAL_H)) {
    delay(1);
    if (lerDados(buffer, 6)) {
      if (verificarCRC(buffer, buffer[2]) && verificarCRC(buffer + 3, buffer[5])) {
        serial = ((uint32_t)buffer[0] << 24) | ((uint32_t)buffer[1] << 16);
        serial |= ((uint32_t)buffer[3] << 8) | buffer[4];
      }
    }
  }
  
  return serial;
}

// ============================================================
// STATUS
// ============================================================

bool SHT3X::sensorOk() {
  return _sensorPresente && (_errosConsecutivos < MAX_ERROS);
}

bool SHT3X::sensorConectado() {
  return _sensorPresente;
}

bool SHT3X::alertaTemperatura() {
  return _alertaTemp;
}

bool SHT3X::alertaUmidade() {
  return _alertaUmid;
}

bool SHT3X::estaEstavel() {
  return _leiturasEstaveis >= 3;
}

uint8_t SHT3X::getErrosConsecutivos() {
  return _errosConsecutivos;
}

unsigned long SHT3X::tempoDesdeUltimaLeitura() {
  return millis() - _ultimaLeituraValidaMs;
}

bool SHT3X::estaMedindo() {
  return _estado != IDLE;
}

// ============================================================
// DEBUG
// ============================================================

void SHT3X::printConfig() {
  Serial.println("=== CONFIG SHT3X ===");
  Serial.print("Endereco: 0x"); Serial.println(_config.endereco, HEX);
  Serial.print("Modo: "); Serial.println(_config.modoMedicao);
  Serial.print("Intervalo: "); Serial.print(_config.intervaloMs); Serial.println(" ms");
  Serial.print("Offset Temp: "); Serial.print(_config.offsetTemperatura); Serial.println(" C");
  Serial.print("Offset Umid: "); Serial.print(_config.offsetUmidade); Serial.println(" %");
  Serial.print("Alpha EMA: "); Serial.println(_config.alphaEMA);
  Serial.print("Alerta Temp: "); Serial.print(_config.tempMinAlerta); 
  Serial.print(" - "); Serial.print(_config.tempMaxAlerta); Serial.println(" C");
  Serial.print("Alerta Umid: "); Serial.print(_config.umidMinAlerta); 
  Serial.print(" - "); Serial.print(_config.umidMaxAlerta); Serial.println(" %");
  Serial.println("====================");
}

void SHT3X::printStatus() {
  Serial.print("Sensor: "); Serial.print(_sensorPresente ? "OK" : "ERRO");
  Serial.print(" | Erros: "); Serial.print(_errosConsecutivos);
  Serial.print(" | Estavel: "); Serial.print(estaEstavel() ? "SIM" : "NAO");
  Serial.print(" | Aquecedor: "); Serial.println(_aquecedorLigado ? "ON" : "OFF");
}

void SHT3X::printDados() {
  if (!_leituraValida) {
    Serial.println("Dados invalidos");
    return;
  }
  
  Serial.print("Temp: "); Serial.print(_temperaturaAtual, 2); Serial.print(" C");
  Serial.print(" | Umid: "); Serial.print(_umidadeAtual, 1); Serial.print(" %");
  Serial.print(" | Orvalho: "); Serial.print(getPontoOrvalho(), 1); Serial.print(" C");
  Serial.print(" | Calor: "); Serial.print(getIndiceCalor(), 1); Serial.print(" C");
  
  if (_alertaTemp) Serial.print(" [ALERTA TEMP]");
  if (_alertaUmid) Serial.print(" [ALERTA UMID]");
  
  Serial.println();
}

#endif // SHT3X_H


// ============================================================
// EXEMPLO DE USO
// ============================================================

#define SDA_PIN 21
#define SCL_PIN 22

SHT3X sensor;

unsigned long ultimoLog = 0;

void setup() {
  Serial.begin(115200);
  delay(100);
  
  // OPCAO 1: Configuracao via struct
  SHT3X_Config config;
  config.endereco = 0x44;           // 0x44 ou 0x45
  config.modoMedicao = 0;           // 0=High, 1=Med, 2=Low
  config.intervaloMs = 500;         // Medicao a cada 500ms
  config.offsetTemperatura = 0.0f;  // Calibracao
  config.offsetUmidade = 0.0f;
  config.alphaEMA = 0.3f;
  config.variacaoMaxTemp = 5.0f;    // Rejeita saltos > 5C
  config.variacaoMaxUmid = 15.0f;   // Rejeita saltos > 15%
  config.tempMinAlerta = 10.0f;     // Alerta se < 10C
  config.tempMaxAlerta = 40.0f;     // Alerta se > 40C
  config.umidMinAlerta = 20.0f;     // Alerta se < 20%
  config.umidMaxAlerta = 80.0f;     // Alerta se > 80%
  
  if (sensor.begin(config, SDA_PIN, SCL_PIN)) {
    Serial.println("SHT3X inicializado!");
    sensor.printConfig();
    
    uint32_t serial = sensor.getNumeroSerie();
    Serial.print("Numero de serie: 0x");
    Serial.println(serial, HEX);
  } else {
    Serial.println("ERRO: SHT3X nao encontrado!");
  }
  
  Serial.println();
}

void loop() {
  sensor.update();
  
  unsigned long agora = millis();
  if (agora - ultimoLog >= 1000) {
    ultimoLog = agora;
    
    if (sensor.sensorOk()) {
      sensor.printDados();
      
      // Ou acesso individual:
      // float temp = sensor.getTemperatura();
      // float umid = sensor.getUmidade();
      // float orvalho = sensor.getPontoOrvalho();
      
      // Ou dados completos:
      // SHT3X_Dados dados = sensor.getDados();
      
    } else {
      Serial.print("ERRO: ");
      Serial.print(sensor.getErrosConsecutivos());
      Serial.println(" erros consecutivos");
    }
  }
}
