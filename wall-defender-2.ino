/*************************************************************************************************************
*******************************BIT ARRAY *********************************************************************
**************************************************************************************************************
mais informações aqui: http://fabianoallex.blogspot.com.br/2015/09/arduino-array-de-bits.html
**************************************************************************************************************/
class BitArray2D {
  private:
    unsigned int _rows;
    unsigned int _columns;
    unsigned int _cols_array; //pra cada 8 colunas, 1 byte é usado 
    byte**       _bits;
  public:
    BitArray2D(unsigned int rows, unsigned int columns){
      _rows       = rows;
      _columns    = columns;
      _cols_array = columns/8 + (_columns%8 ? 1 : 0) + 1; //divide por 8 o número de colunas
      _bits = (byte **)malloc(_rows * sizeof(byte *));
      for(int i=0;i<_rows;i++){ _bits[i] = (byte *)malloc(  _cols_array  *  sizeof(byte)); } //cria varios arrays
      clear();
    }
    unsigned int rows(){ return _rows; }
    unsigned int columns(){ return _columns; }
    void clear() { 
      for(int i=0;i<_rows;i++){      
        for(int j=0; j<_cols_array;j++) { _bits[i][j] = B00000000; }       
      }   
    }
    void write(unsigned int row, unsigned int column, int value){
      byte b = _bits[row][ column/8 + (column%8 ? 1 : 0) ];
      unsigned int bit = column%8;    
      if (value) { b |= (1 << bit); } else { b &= ~(1 << bit);  }
      _bits[row][ column/8 + (column%8 ? 1 : 0) ] = b;
    }
    void write(byte value){
      for(int i=0;i<_rows;i++){      
        for(int j=0; j<_cols_array;j++) {      
          _bits[i][j] = value ? B11111111 : B00000000;     
        }       
      }  
    }
    int read(unsigned int row, unsigned int column){
      byte b = _bits[row][ column/8 + (column%8 ? 1 : 0) ];
      unsigned int bit = column%8;    
      return (b & (1 << bit)) != 0;
    }
    void toggle(unsigned int row, unsigned int column){ write(row, column, !read(row, column)); }
    void toggle(){ for(int i=0;i<_rows;i++){      for(int j=0; j<_columns;j++) {      toggle(i,j);   }   }   }
};
    
/*************************************************************************************************************
*******************************FIM BIT ARRAY *****************************************************************
**************************************************************************************************************/
   
/*************************************************************************************************************
************************************CLASSE UNIQUE RANDOM******************************************************
**************************************************************************************************************
mais informações aqui: http://fabianoallex.blogspot.com.br/2015/09/arduino-numeros-aleatorio-repetidos-e.html
*************************************************************************************************************/
class UniqueRandom{
  private:
    int _index;
    int _min;
    int _max;
    int _size;
    int* _list;
    void _init(int min, int max) {
      _list = 0; 
      if (min < max) { _min = min; _max = max; } else { _min = max; _max = min; }
      _size = _max - _min; 
      _index = 0;
    }    
  public:
    UniqueRandom(int max)           { _init(0,   max); randomize(); } //construtor com 1 parametro
    UniqueRandom(int min, int max)  { _init(min, max); randomize(); } //construtor com 2 parametros
    void randomize() {
      _index = 0;
      if (_list == 0) { _list = (int*) malloc(size() * sizeof(int)); }  
      for (int i=0; i<size(); i++) {   _list[i] = _min+i;  }   //preenche a lista do menor ao maior valor
      //embaralha a lista
      for (int i=0; i<size(); i++) {  
        int r = random(0, size());     //sorteia uma posição qualquer
        int aux = _list[i];               
        _list[i] = _list[r];
        _list[r] = aux;
      }
    }
    int next() {                                  //retorna o proximo numero da lista
      int n = _list[_index++];
      if (_index >= size() ) { _index = 0;} //após recuper o ultimo numero, recomeça na posicao 0
      return n;
    }
    void first() { _index = 0; }
    boolean eof() { return size() == _index+1; }
    int size() { return _size; }
    int getIndex() {  return _index; }
    ~UniqueRandom(){ free ( _list ); }  //destrutor
};
/*************************************************************************************************************
************************************FIM CLASSE UNIQUE RANDOM**************************************************
*************************************************************************************************************/
   
/*************************************************************************************************************
*******************************LEDCONTROL ALTERADA************************************************************
mais informações: http://fabianoallex.blogspot.com.br/2015/09/arduino-alteracoes-na-biblioteca.html
**************************************************************************************************************/
//the opcodes for the MAX7221 and MAX7219
#define OP_DECODEMODE  9
#define OP_INTENSITY   10
#define OP_SCANLIMIT   11
#define OP_SHUTDOWN    12
#define OP_DISPLAYTEST 15
class LedControl {
  private :
    byte spidata[16];
    byte * status;
    int SPI_MOSI;
    int SPI_CLK;
    int SPI_CS;
    int maxDevices;
    int _auto_send;
    void spiTransfer(int addr, volatile byte opcode, volatile byte data) {
      int offset   = addr*2;
      int maxbytes = maxDevices*2;
      for(int i=0;i<maxbytes;i++)  { spidata[i]=(byte)0; }
      spidata[offset+1] = opcode;
      spidata[offset]   = data;
      digitalWrite(SPI_CS,LOW);
      for(int i=maxbytes;i>0;i--) { shiftOut(SPI_MOSI,SPI_CLK,MSBFIRST,spidata[i-1]); }
      digitalWrite(SPI_CS,HIGH);
    }
  public:
    LedControl(int dataPin, int clkPin, int csPin, int numDevices) {
      _auto_send  = true;
      SPI_MOSI    = dataPin;
      SPI_CLK     = clkPin;
      SPI_CS      = csPin;
      maxDevices  = numDevices;
      pinMode(SPI_MOSI, OUTPUT);
      pinMode(SPI_CLK,  OUTPUT);
      pinMode(SPI_CS,   OUTPUT);
      digitalWrite(SPI_CS, HIGH);
      status      = new byte[maxDevices * 8]; //instancia o array de acordo com a quantia de displays usados
      for(int i=0;i<maxDevices * 8 ;i++) { status[i]=0x00; }
      for(int i=0;i<maxDevices;i++) {
        spiTransfer(i, OP_DISPLAYTEST,0);
        setScanLimit(i, 7);               //scanlimit is set to max on startup
        spiTransfer(i, OP_DECODEMODE,0);  //decode is done in source
        clearDisplay(i);
        shutdown(i,true);                 //we go into shutdown-mode on startup
      }
    }
    void startWrite() {  _auto_send = false;  };
    void send() {
      for (int j=0; j<maxDevices; j++) {
        int offset = j*8;
        for(int i=0;i<8;i++) { spiTransfer(j, i+1, status[offset+i]); }
      }
      _auto_send = true;
    }
    int getDeviceCount(){ return maxDevices; }
    void shutdown(int addr, bool b){
      if(addr<0 || addr>=maxDevices) return;
      spiTransfer(addr, OP_SHUTDOWN, b ? 0 : 1);
    }
    void setScanLimit(int addr, int limit){
      if(addr<0 || addr>=maxDevices) return;
      if(limit>=0 && limit<8) spiTransfer(addr, OP_SCANLIMIT,limit);
    }
    void setIntensity(int addr, int intensity) {
      if(addr<0 || addr>=maxDevices)   {  return;                                    }
      if(intensity>=0 && intensity<16) {  spiTransfer(addr, OP_INTENSITY, intensity); }
    }
    void clearDisplay(int addr){
      if(addr<0 || addr>=maxDevices) return;
      int offset = addr*8;
      for(int i=0;i<8;i++) {
        status[offset+i] = 0;
        if (_auto_send) { spiTransfer(addr, i+1, status[offset+i]); }
      }
    }
    void setLed(int addr, int row, int column, boolean state) {
      if(addr<0 || addr>=maxDevices)             { return; }
      if(row<0 || row>7 || column<0 || column>7) { return; }
      int offset = addr*8;
      byte val = B10000000 >> column;
      if(state) { status[offset+row] = status[offset+row] | val; } else { val=~val; status[offset+row] = status[offset+row]&val; }
      if (_auto_send) { spiTransfer(addr, row+1, status[offset+row]); }
    }
    void setRow(int addr, int row, byte value) {
      if(addr<0 || addr>=maxDevices) return;
      if(row<0 || row>7) return;
      int offset = addr*8;
      status[offset+row] = value;
      if (_auto_send) { spiTransfer(addr, row+1, status[offset+row]); }
    }
    void setColumn(int addr, int col, byte value) {
      if(addr<0 || addr>=maxDevices) return;
      if(col<0 || col>7)             return;
      byte val;
      for(int row=0; row<8; row++) {
        val=value >> (7-row);
        val=val & 0x01;
        setLed(addr,row,col,val);
      }
    }
};
/*************************************************************************************************************
*******************************FIM LEDCONTROL ALTERADA********************************************************
**************************************************************************************************************/
   
/*************************************************************************************************************
************************************CLASSE ROTARY ENCODER*****************************************************
mais informações: http://fabianoallex.blogspot.com.br/2016/05/arduino-rotary-encoder.html
*************************************************************************************************************/
#define ROTARY_NO_BUTTON     255
       
struct RotaryEncoderLimits{
  int min;
  int max;
};
       
class RotaryEncoder {
  private:
    byte _pin_clk;
    byte _pin_dt;
    byte _pin_sw;
    volatile byte _num_results;
    volatile int _result;
    volatile int * _results;
    byte _index_result;
    RotaryEncoderLimits * _limits;
    byte _a : 1;
    byte _b : 1;
  public:
    RotaryEncoder(byte pin_clk, byte pin_dt, byte pin_sw = ROTARY_NO_BUTTON, byte num_results=1, RotaryEncoderLimits * limits=0){   //parametro do botao opcional
      _pin_clk = pin_clk;
      _pin_dt = pin_dt;
      _pin_sw = pin_sw;
      pinMode(_pin_clk, INPUT);
      pinMode(_pin_dt, INPUT);
      if (_pin_sw != ROTARY_NO_BUTTON){ 
        pinMode(_pin_sw, INPUT); 
        digitalWrite(_pin_sw, HIGH);
      }
      if (num_results == 0) { num_results = 1; }
      _num_results = num_results;
      _results = new int[_num_results];
      for (int i; i<_num_results; i++){ _results[i] = (limits) ? limits[i].min : 0; }
      _index_result = 0;
      _limits = limits;
      _a = false;
      _b = false;
    }
    byte getIndex() { return _index_result; }
    void next()     { _index_result++; if (_index_result >= _num_results) { _index_result = 0; } } 
    void update_a() {
      _result = 0;
      delay (1);
      if( digitalRead(_pin_clk) != _a ) { 
        _a = !_a;
        if ( _a && !_b ) { _result = -1; }
      }
      if (_results[_index_result]+_result >= _limits[_index_result].min && 
          _results[_index_result]+_result <= _limits[_index_result].max ) {
        _results[_index_result] += _result;
      }
    }
    void update_b() {
      _result = 0;
      delay (1);
      if( digitalRead(_pin_dt) != _b ) {  
        _b = !_b;
        if ( _b && !_a ) { _result = +1; }
      }
      if (_results[_index_result]+_result >= _limits[_index_result].min && 
          _results[_index_result]+_result <= _limits[_index_result].max ) {
        _results[_index_result] += _result;
      }
    }
    int read(){ return _result; }                                        //retorn -1, 0 ou 1.
    int getValue(int index=-1) {                                         //retorna o valor da variável corrente ou a passada como parametro
      if (index < 0 ){ return _results[_index_result]; }
      return _results[index];
    }
    void setValue(int value, int index=-1){ _results[ (index==-1) ? _index_result : index] = value; }         //caso a variável inicializa em determinado valor diferente de zero, utilizar esse método.
    int buttonRead(){ return (_pin_sw == ROTARY_NO_BUTTON) ? LOW : digitalRead(_pin_sw); }
};
/*************************************************************************************************************
************************************FIM CLASSE ROTARY ENCODER*************************************************
*************************************************************************************************************/
   
/*************************************************************************************************************
*******************************CLASSE WALL DEFENDER GAME******************************************************
**************************************************************************************************************/
struct Position {
  int lin ;  //maximo 15
  int col ;  //maximo 15, mas será no máximo 6 EM DISPLAY 16X2 E TERÁ 12 EM DISPLAY 20X4. PARA MAIS DE 15 LINHAS. ALTERAR AQUI
};
 
const int WALL_DEFENDER_MAX_WALL_HIGH = 15;  //quantidade maxima de inimigos na tela
const int WALL_DEFENDER_MAX_ENEMY = 10;  //quantidade maxima de obstáculos na tela
const int WALL_DEFENDER_TIME_INIT = 700; //tempo entre deslocamento dos inimigos
const int WALL_DEFENDER_TIME_INC  = 10;  //incremento da velocidade
enum WallDefenderGameStatus { WALL_GAME_ON, WALL_GAME_OVER };
enum WallShootStatus {WALL_SHOOT_LEFT, WALL_SHOOT_RIGHT, WALL_SHOOT_STOP};
enum Direction { DIR_STOP, DIR_TOP, DIR_LEFT, DIR_BOTTOM, DIR_RIGHT};
 
struct Enemy{
  Position pos;
  Direction dir;
};
 
class WallDefenderGame {
  private:
    BitArray2D * _display;
    Position _position;
    Position _shoot;
    WallShootStatus _shootStatus;
    Position _wall[WALL_DEFENDER_MAX_WALL_HIGH];
    Enemy _enemys[WALL_DEFENDER_MAX_ENEMY];
    byte _lengthWall;
    byte _controlEnemys : 2;  //máximo = 3 B11
    byte _countUpdate : 3;       //maximo = 5 B101
    Direction _direction;
    unsigned long _last_millis;
    unsigned long _last_millis_player;
    unsigned long _last_millis_shoot;
    int _time;
    int _score;
    int _lengthEnemy;
    WallDefenderGameStatus _wallDefenderGameStatus;
    UniqueRandom * _ur;  //utilizado no game over
    void _gameOver(){ 
      _wallDefenderGameStatus = WALL_GAME_OVER; 
      _direction   = DIR_STOP;
      _time = 20;
    }
    void _inc_speed(){
      _score++;
      _time -= WALL_DEFENDER_TIME_INC;
    }
    void _generateWall() {
      _lengthWall = 0;
      for (int i=0;i<(_display->rows()-2)*2;i++) {
        _wall[i].lin = i<(_display->rows()-2) ? i+1 : i+1-_display->rows()+2;
        _wall[i].col = i<(_display->rows()-2) ?  _display->columns()/2-1 : _display->columns()/2;
        _lengthWall++;
      }
    }
    void _start(){
      for (int i=0; i<WALL_DEFENDER_MAX_ENEMY; i++){ 
        _enemys[i].dir == DIR_STOP; 
        _enemys[i].pos.lin=0;
        _enemys[i].pos.col=0;
      }
      _score  = 0;
      _time = WALL_DEFENDER_TIME_INIT;
      _last_millis  = 0;
      _last_millis_player = 0;
      _position.lin = 0; 
      _position.col = _display->columns() / 2 + 1;
      _direction = DIR_STOP;
      _wallDefenderGameStatus = WALL_GAME_ON;
      _shootStatus = WALL_SHOOT_STOP;
      _generateWall();
      
      Serial.println("a");
    }
    void _generateEnemys() {  
      Serial.println(_lengthEnemy);
      if (_controlEnemys == 0 && _lengthEnemy < WALL_DEFENDER_MAX_ENEMY){
        _lengthEnemy++;
         
        int r1 = random(0, _display->rows());
        int r2 = random(0, 2);
          
        for (int i=0; i<WALL_DEFENDER_MAX_ENEMY; i++){
          if (_enemys[i].dir == DIR_STOP){
            _enemys[i].dir     = (r2 == 0) ? DIR_RIGHT : DIR_LEFT;
            _enemys[i].pos.col = (r2 == 0) ? 0 : _display->columns()-1;
            _enemys[i].pos.lin = r1;
            break;
          }
        }
      }
      _controlEnemys = (_controlEnemys < 2) ? _controlEnemys+1 : 0;  //a cada 4 atualizacoes gera novos obstáculos
    }
    void _runShoot(){
      if (_shootStatus == WALL_SHOOT_LEFT ) {  if (_shoot.col == 0)                     { _shootStatus = WALL_SHOOT_STOP; } else { _shoot.col--; } }
      if (_shootStatus == WALL_SHOOT_RIGHT) {  if (_shoot.col == _display->columns()-1) { _shootStatus = WALL_SHOOT_STOP; } else { _shoot.col++; }  }
    }
    void _runGameOver(){
      int r   = _ur->next();
      int lin = (r / _display->columns());
      int col = (r % _display->columns());
      _display->write(lin, col, HIGH );
      if ( _ur->getIndex() == (_ur->size()-1) ) {
        _ur->randomize();
        _ur->first();
        _start(); 
        Serial.println("teste");
      }
    }
    void _runPlayer(){
      if (_direction == DIR_TOP    && _position.lin > 0 )                     { _position.lin--;  }
      if (_direction == DIR_BOTTOM && _position.lin < _display->rows()-1 )    { _position.lin++;  }
      if (_direction == DIR_LEFT   && _position.col > 0 )                     { _position.col--;  }
      if (_direction == DIR_RIGHT  && _position.col < _display->columns()-1 ) { _position.col++;  }
        
      _direction = DIR_STOP; //depois de mover, mantem parado.
    }
    void _runEnemys() {
      /*move os inimigos*/
      byte contRemove = 0;
      for (int i=0; i<WALL_DEFENDER_MAX_ENEMY; i++) {
        if (_enemys[i].dir == DIR_RIGHT && _enemys[i].pos.col == _display->columns()-1){ contRemove++; _enemys[i].dir = DIR_STOP; }
        if (_enemys[i].dir == DIR_LEFT  && _enemys[i].pos.col == 0)                    { contRemove++; _enemys[i].dir = DIR_STOP; }
        if (_enemys[i].dir != DIR_STOP ) {  _enemys[i].dir == DIR_LEFT ? _enemys[i].pos.col-- : _enemys[i].pos.col++; }
      }
      if (contRemove > 0){ _lengthEnemy -= contRemove;  }
      _generateEnemys(); 
      _countUpdate++;
      if (_countUpdate >= 5){
        _inc_speed();
        _countUpdate = 0;
      }
    }
     
    void _updateDisplay(){
      //verifica se o tiro colidiu com o inimigo
      for (int i=0; i<WALL_DEFENDER_MAX_ENEMY; i++){
        if (_enemys[i].dir != DIR_STOP ) {  
          if (_enemys[i].pos.lin == _shoot.lin    && _enemys[i].pos.col == _shoot.col)    { _lengthEnemy--; _enemys[i].dir = DIR_STOP;  _shootStatus = WALL_SHOOT_STOP;}
          if (_enemys[i].pos.lin == _position.lin && _enemys[i].pos.col == _position.col) { _gameOver(); }
        }
      }
      //verifica se colidiu na parede
      for (int i=0; i<WALL_DEFENDER_MAX_ENEMY; i++){
        for (int p=0; p<_lengthWall; p++) {
          if (_enemys[i].dir != DIR_STOP ) {
            if (_enemys[i].pos.lin == _wall[p].lin && _enemys[i].pos.col == _wall[p].col) { _gameOver(); }
          }
        }
      }
      if (_wallDefenderGameStatus == WALL_GAME_ON) { _display->clear();  }
      for (int lin=0; lin<_display->rows(); lin++) {
        for (int col=0; col<_display->columns(); col++) {
          for (int p=0; p<_lengthWall; p++){
            boolean val = _wall[p].col==col && _wall[p].lin==lin;
            if (val) { _display->write( lin, col,  val ); break;}
          }
          for (int p=0; p<WALL_DEFENDER_MAX_ENEMY; p++) {
            if (_enemys[p].dir != DIR_STOP){
              boolean val = _enemys[p].pos.col==col && _enemys[p].pos.lin==lin;
              if (val) { _display->write( lin, col,  val ); break;}
            }
          }
        }
      }
      _display->write(_position.lin, _position.col, HIGH);
      if (_shootStatus != WALL_SHOOT_STOP) { _display->write(_shoot.lin, _shoot.col, HIGH); }
    }
  public:
    WallDefenderGame(BitArray2D * display){ 
      _display = display;
      _ur = new UniqueRandom( _display->rows() * _display->columns() );
      _lengthWall = 0;
      _lengthEnemy = 0;
      _start();
    }
    void left()   { _direction = DIR_LEFT;   }
    void right()  { _direction = DIR_RIGHT;  }
    void top()    { _direction = DIR_TOP;    }
    void bottom() { _direction = DIR_BOTTOM; }
    int getScore(){ return _score; }
    Position getPositionPlayer() { return _position; }
    void shoot(){
      if (_shootStatus == WALL_SHOOT_STOP){
        _shoot.lin   = _position.lin;
        _shoot.col   = _position.col;
        _shootStatus = (_shoot.col <= _display->columns() / 2-1) ? WALL_SHOOT_LEFT : WALL_SHOOT_RIGHT;
      }
    }
    int update(){
      int r = false;
      if (millis() - _last_millis > _time) {
        r = true;
        _last_millis = millis();  
        if (_wallDefenderGameStatus == WALL_GAME_ON)   { _runEnemys(); } 
        if (_wallDefenderGameStatus == WALL_GAME_OVER) { _runGameOver(); }
      }
      if (millis() - _last_millis_player > 50){
        r = true;
        _last_millis_player = millis();
        if (_wallDefenderGameStatus == WALL_GAME_ON)   { _runPlayer(); } 
        if (_wallDefenderGameStatus == WALL_GAME_OVER) { _runGameOver(); }
      }
      if (millis() - _last_millis_shoot > 50){
        r = true;
        _last_millis_shoot = millis();
        if (_wallDefenderGameStatus == WALL_GAME_ON)   { _runShoot(); } 
        if (_wallDefenderGameStatus == WALL_GAME_OVER) { _runGameOver(); }
      }
      if (r) { _updateDisplay(); }
      return r; //r-->indica se houve mudança no display
    }
};
/*************************************************************************************************************
*******************************FIM CLASSE WALL DEFENDER GAME**************************************************
**************************************************************************************************************/
 
/*************************************************************************************************************
*******************************DISPLAY************************************************************************
**************************************************************************************************************/
 
/*
rotation indica qual parte do display estará para cima
 
         TOP
L  . . . . . . . . R
E  . . . . . . . . I
F  . . . . . . . . G
T  . . . . . . . . H
   . . . . . . . . T
   . . . . . . . .
   . . . . . . . .
   . . . . . . . .   
       BOTTOM
     
*/
enum Rotation {TOP, LEFT, BOTTOM, RIGHT};
 
struct Display {
  int index;
  Position position; 
  Rotation rotation;
};
 
/*************************************************************************************************************
*******************************FIM DISPLAY********************************************************************
**************************************************************************************************************/
 
/*************************************************************************************************************
*******************************DECLARACAO DOS OBJETOS*********************************************************
**************************************************************************************************************/
/*
 tamanho display real
 * * * * * * * * * * * * * * * *
 * * * * * * * * * * * * * * * *
 * * * * * * * * * * * * * * * *
 * * * * * * * * * * * * * * * *
 * * * * * * * * * * * * * * * *
 * * * * * * * * * * * * * * * *
 * * * * * * * * * * * * * * * *
 * * * * * * * * * * * * * * * * 
*/
   
const int LINHAS  = 8;
const int COLUNAS = 16;

Display displays_8x8[] = {
  {0, {0,0}, LEFT},
  {1, {0,8}, RIGHT}
};
    
BitArray2D ba(LINHAS, COLUNAS); //8 linhas e 16 colunas, usada para armazenar o estado do display
WallDefenderGame wall(&ba);
   
RotaryEncoderLimits lim[] = { {-1000,1000} };  //limites máximos e mínimos que as variaveis podem atingir
RotaryEncoder       re(A0, A1, 4, 1, lim);  //pino clk, pino dt, pino sw, qtd variaveis, limites

LedControl lc=LedControl(10,12,11,2);  //pin 10: DataIn ; pin 12: CLK ;  pin 11: LOAD   --- 2 display 8x8

   
/*************************************************************************************************************
*******************************FIM DECLARACAO DOS OBJETOS*****************************************************
**************************************************************************************************************/
     
/*************************************************************************************************************
*******************************TRATAMENTO DAS INTERRUPÇÕES****************************************************
**************************************************************************************************************/
//interrupções dos pinos A0 e A1 via Pin Change Interrupt
ISR(PCINT1_vect) {
  volatile static byte lastVal_a0 = LOW;
  volatile static byte lastVal_a1 = LOW;
  byte val_a0 = digitalRead(A0);
  byte val_a1 = digitalRead(A1);
  if (lastVal_a0 != val_a0){ re.update_a(); lastVal_a0 = val_a0; }
  if (lastVal_a1 != val_a1){ re.update_b(); lastVal_a1 = val_a1; }
}
    
void setup_interrupts(){
  //-----PCI - Pin Change Interrupt ----
  pinMode(A0,INPUT);   // set Pin as Input (default)
  digitalWrite(A0,HIGH);  // enable pullup resistor
  pinMode(A1,INPUT);   // set Pin as Input (default)
  digitalWrite(A1,HIGH);  // enable pullup resistor
  cli();
  PCICR |= 0b00000010; // habilita a porta C - Pin Change Interrupts
  PCMSK1 |= 0b00000011; // habilita interrupção da porta c nos pinos: PCINT8 (A0) e PCINT9(A1)
  sei();
}
/*************************************************************************************************************
*******************************FIM TRATAMENTO DAS INTERRUPÇÕES****************************************************
**************************************************************************************************************/
void update_display() {
  lc.startWrite();
   
  for (int lin=0; lin<ba.rows(); lin++) {
    for (int col=0; col<ba.columns(); col++) {
      for (int i=0; i<sizeof(displays_8x8)/sizeof(Display); i++) {
        int l = lin - displays_8x8[i].position.lin;
        int c = col - displays_8x8[i].position.col;
        if (l>=0 && l<=7 && c>=0 && c<=7) {
          if (displays_8x8[i].rotation == BOTTOM) {            c=7-c; l=7-l;   }
          if (displays_8x8[i].rotation == LEFT)   { int aux=c; c=l;   l=7-aux; }
          if (displays_8x8[i].rotation == RIGHT)  { int aux=l; l=c;   c=7-aux; }
          lc.setLed(displays_8x8[i].index, l, c, ba.read(lin, col) );
        }
      }
    }
  }
   
  lc.send();
}
 
 
void setup() { 
  Serial.begin(9600);
  
  lc.shutdown(0,false);
  lc.setIntensity(0,1);
  lc.clearDisplay(0);  
  lc.shutdown(1,false);
  lc.setIntensity(1,1);
  lc.clearDisplay(1);  
  
  setup_interrupts();
  randomSeed(analogRead(A2));
  re.setValue(0, 0);  //inicializa o rotary em 0
}
      
void loop() {
  static int val_encoder = 0;
  static boolean change = false;
     
  //anti-horario
  if (re.getValue(0) < val_encoder && !change && wall.getPositionPlayer().lin > 0  && wall.getPositionPlayer().col == 9){ wall.top();    change = true; }
  if (re.getValue(0) < val_encoder && !change && wall.getPositionPlayer().lin == 0 && wall.getPositionPlayer().col >= 7){ wall.left();   change = true; }
  if (re.getValue(0) < val_encoder && !change && wall.getPositionPlayer().lin < 7  && wall.getPositionPlayer().col == 6){ wall.bottom(); change = true; }  
  if (re.getValue(0) < val_encoder && !change && wall.getPositionPlayer().lin == 7 && wall.getPositionPlayer().col <= 9){ wall.right();  change = true; }
  //horario
  if (re.getValue(0) > val_encoder && !change && wall.getPositionPlayer().lin > 0  && wall.getPositionPlayer().col == 6){ wall.top();    change = true; }  
  if (re.getValue(0) > val_encoder && !change && wall.getPositionPlayer().lin == 7 && wall.getPositionPlayer().col >= 7){ wall.left();   change = true; }
  if (re.getValue(0) > val_encoder && !change && wall.getPositionPlayer().lin < 7  && wall.getPositionPlayer().col == 9){ wall.bottom(); change = true; }
  if (re.getValue(0) > val_encoder && !change && wall.getPositionPlayer().lin == 0 && wall.getPositionPlayer().col <= 8){ wall.right();  change = true; }
   
  val_encoder = re.getValue(0);
  if ( wall.update() ) { 
    change = false;
    update_display(); 
  }  
     
  //controla o click do botao do enconder
  static byte b = HIGH; //pra ler apenas uma vez o botao ao pressionar
  if( re.buttonRead() == LOW && b != re.buttonRead() ) {
    wall.shoot();        //dispara o tiro
    delay(50);           //debounce meia boca
  }
  b = re.buttonRead();
} 