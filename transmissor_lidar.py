import serial
import socket
import math

# --- CONFIGURAÇÃO DE REDE ---
IP_NOTEBOOK = '192.168.18.45' # Substitua pelo IP do seu Windows
PORTA_UDP = 5005
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# --- CONFIGURAÇÃO SERIAL ---
# Porta ttyAMA0 a 115200 baud conforme sua captura
ser = serial.Serial('/dev/ttyAMA0', 115200, timeout=0.1)

def transmitir_atlas_v3():
    ser.flushInput()
    print("Protocolo 36-bytes/8-amostras detectado. Iniciando mapeamento...")
    
    while True:
        # Sincronização com o Header 55 AA
        if ser.read(1) == b'\x55':
            if ser.read(1) == b'\xAA':
                # Lê os 34 bytes restantes do frame de 36 bytes
                packet = ser.read(34)
                if len(packet) < 34: continue
                
                # Byte 1 do buffer (Byte 3 total): Quantidade de amostras
                num_amostras = packet[1] # Deve ser 0x08
                
                # Bytes 4-5 (Total 6-7): Ângulo Inicial (Little Endian)
                raw_start = (packet[5] << 8) | packet[4]
                # Bytes 30-31 (Total 32-33): Ângulo Final (Little Endian)
                raw_end = (packet[31] << 8) | packet[30]
                
                # Conversão para Graus (Ajuste a escala baseada no seu protocolo 0x4E0C/0xC0A4)
                # Muitos sensores OEM usam (Raw / 64.0) para obter graus decimais
                angle_start = (raw_start / 64.0) % 360
                angle_end = (raw_end / 64.0) % 360
                
                # Interpolação angular entre as 8 amostras
                diff = (angle_end - angle_start + 360) % 360
                step = diff / (num_amostras - 1) if num_amostras > 1 else 0

                # Processa os 8 pontos (Bytes 6 a 29 do buffer)
                for i in range(num_amostras):
                    offset = 6 + (i * 3)
                    dist_lsb = packet[offset]
                    dist_msb = packet[offset + 1]
                    intensity = packet[offset + 2]
                    
                    distancia_mm = (dist_msb << 8) | dist_lsb
                    
                    # Filtro de confiança e distância útil para o ATLAS
                    if intensity > 5 and 50 < distancia_mm < 20000:
                        angle_ponto = math.radians((angle_start + (i * step)) % 360)
                        
                        # Conversão para Cartesiano para o Visualizador 2D
                        x = int(distancia_mm * math.cos(angle_ponto))
                        y = int(distancia_mm * math.sin(angle_ponto))
                        
                        sock.sendto(f"{x},{y}".encode(), (IP_NOTEBOOK, PORTA_UDP))

if __name__ == "__main__":
    transmitir_atlas_v3()
