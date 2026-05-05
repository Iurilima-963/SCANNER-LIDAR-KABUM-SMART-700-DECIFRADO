import socket
import pygame
import sys
import math

# --- CONFIGURAÇÃO DE REDE ---
IP_OUVIR = '0.0.0.0'
PORTA_UDP = 5005
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((IP_OUVIR, PORTA_UDP))
sock.setblocking(False)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1048576)

# --- CONFIGURAÇÃO VISUAL ---
LARGURA, ALTURA = 1100, 900
pygame.init()
tela = pygame.display.set_mode((LARGURA, ALTURA), pygame.RESIZABLE)
pygame.display.set_caption("ATLAS SLAM PRO - OCCUPANCY GRID MODE")
relogio = pygame.time.Clock()

# Cores Atualizadas (Baseadas na image_098823.png)
COR_CINZA_VAZIO = (80, 80, 80)    # Área não explorada
COR_BRANCO_LIVRE = (245, 245, 245) # Área segura para andar
COR_PAREDE = (0, 180, 120)       # Obstáculo confirmado (Esmeralda)
COR_ROBO = (255, 50, 50)
COR_SCAN_VIVO = (255, 255, 255)
COR_HUD = (0, 200, 255)
COR_PERIGO = (255, 30, 30)

# --- ENGINE SLAM ---
robo_x, robo_y = 0.0, 0.0  
angulo_robo = 0.0         
# pontos_mapa agora guarda [tipo, conf] -> tipo 0: Livre(Branco), tipo 1: Parede(Verde)
pontos_mapa = {} 
distancias_atlas = {"FRENTE": 0, "TRAS": 0, "ESQ": 0, "DIR": 0}

GRID_SIZE = 6          
CONF_MIN_EXIBIR = 15   
CONF_MAX = 100         

escala = 0.3
offset_x, offset_y = 0, 0
arrastando = False

def rotacionar(x, y, angulo):
    rad = math.radians(angulo)
    c, s = math.cos(rad), math.sin(rad)
    return x * c - y * s, x * s + y * c

def obter_score_alinhamento(tx, ty, tang, scan_bruto):
    score = 0
    for lx, ly in scan_bruto[::4]:
        rx, ry = rotacionar(-lx, ly, tang)
        gx = int((rx + tx) // GRID_SIZE)
        gy = int((ry + ty) // GRID_SIZE)
        info = pontos_mapa.get((gx, gy))
        if info and info[0] == 1 and info[1] >= 15: # Só alinha com paredes
            score += info[1]
    return score

def limpar_caminho_branco(x0, y0, x1, y1):
    """ Algoritmo de Bresenham para marcar células como LIVRES (Branco) """
    gx0, gy0 = int(x0 // GRID_SIZE), int(y0 // GRID_SIZE)
    gx1, gy1 = int(x1 // GRID_SIZE), int(y1 // GRID_SIZE)
    dx = abs(gx1 - gx0); dy = abs(gy1 - gy0)
    sx = 1 if gx0 < gx1 else -1; sy = 1 if gy0 < gy1 else -1
    err = dx - dy
    cx, cy = gx0, gy0
    
    while (cx, cy) != (gx1, gy1):
        # Se a célula não é uma parede confirmada, vira zona branca
        info = pontos_mapa.get((cx, cy))
        if not info or info[0] == 0:
            pontos_mapa[(cx, cy)] = [0, 100]
        
        e2 = 2 * err
        if e2 > -dy: err -= dy; cx += sx
        if e2 < dx: err += dx; cy += sy
        if abs(cx-gx0) > 150 or abs(cy-gy0) > 150: break # Limite de segurança

def processar_slam():
    global robo_x, robo_y, angulo_robo
    dados_brutos = []
    
    while True:
        try:
            data, _ = sock.recvfrom(16384)
            for msg in data.decode().split(';'):
                p = msg.split(',')
                if len(p) == 2: dados_brutos.append((int(p[0]), int(p[1])))
        except: break

    if not dados_brutos: return []

    if len(pontos_mapa) > 200:
        melhor_x, melhor_y, melhor_ang = robo_x, robo_y, angulo_robo
        max_score = obter_score_alinhamento(robo_x, robo_y, angulo_robo, dados_brutos)
        for da in [-1.5, 0, 1.5]:
            for dx in [-GRID_SIZE, 0, GRID_SIZE]:
                for dy in [-GRID_SIZE, 0, GRID_SIZE]:
                    s = obter_score_alinhamento(robo_x + dx, robo_y + dy, angulo_robo + da, dados_brutos)
                    if s > max_score:
                        max_score = s
                        melhor_x, melhor_y, melhor_ang = robo_x + dx, robo_y + dy, angulo_robo + da
        robo_x, robo_y, angulo_robo = melhor_x, melhor_y, melhor_ang

    calcular_distancias_setores(dados_brutos)
    
    scan_corrigido = []
    for lx, ly in dados_brutos:
        rx, ry = rotacionar(-lx, ly, angulo_robo)
        gx_world, gy_world = rx + robo_x, ry + robo_y
        scan_corrigido.append((gx_world, gy_world))
        
        # 1. Marcar parede
        gx, gy = int(gx_world // GRID_SIZE), int(gy_world // GRID_SIZE)
        info = pontos_mapa.get((gx, gy), [1, 0])
        pontos_mapa[(gx, gy)] = [1, min(info[1] + 10, CONF_MAX)]
        
        # 2. Marcar caminho como livre (Branco)
        limpar_caminho_branco(robo_x, robo_y, gx_world, gy_world)
        
    return scan_corrigido

def calcular_distancias_setores(dados):
    global distancias_atlas
    dists = {"FRENTE": 10000, "TRAS": 10000, "ESQ": 10000, "DIR": 10000}
    for lx, ly in dados:
        d = math.sqrt(lx**2 + ly**2)
        if d < 40: continue
        ang = (math.degrees(math.atan2(ly, -lx)) + 180) % 360
        if ang < 45 or ang > 315: dists["FRENTE"] = min(dists["FRENTE"], d)
        elif 45 <= ang < 135:     dists["ESQ"] = min(dists["ESQ"], d)
        elif 135 <= ang < 225:    dists["TRAS"] = min(dists["TRAS"], d)
        elif 225 <= ang <= 315:   dists["DIR"] = min(dists["DIR"], d)
    distancias_atlas = dists

def world_to_screen(wx, wy):
    dx, dy = wx - robo_x, wy - robo_y
    rx, ry = rotacionar(dx, dy, -angulo_robo)
    sx = int((LARGURA // 2 + offset_x) + (rx * escala))
    sy = int((ALTURA // 2 + offset_y) - (ry * escala))
    return sx, sy

def desenhar_hud_visao():
    f_titulo = pygame.font.SysFont("Consolas", 16, bold=True)
    f_valor = pygame.font.SysFont("Consolas", 26, bold=True)
    setores = [
        ("FRENTE", distancias_atlas["FRENTE"], (LARGURA//2 - 65, 30)),
        ("TRAS",   distancias_atlas["TRAS"],   (LARGURA//2 - 65, ALTURA - 80)),
        ("ESQ",    distancias_atlas["ESQ"],    (40, ALTURA//2 - 30)),
        ("DIR",    distancias_atlas["DIR"],    (LARGURA - 180, ALTURA//2 - 30))
    ]
    for nome, dist, pos in setores:
        cor = COR_PERIGO if dist < 500 else COR_HUD
        tela.blit(f_titulo.render(nome, True, cor), pos)
        val_txt = f"{int(dist)}mm" if dist < 9000 else "LIVRE"
        tela.blit(f_valor.render(val_txt, True, (255, 255, 255)), (pos[0], pos[1] + 20))

# --- LOOP PRINCIPAL ---
while True:
    tela.fill(COR_CINZA_VAZIO) # Fundo Cinza conforme solicitado
    
    for ev in pygame.event.get():
        if ev.type == pygame.QUIT: pygame.quit(); sys.exit()
        if ev.type == pygame.MOUSEBUTTONDOWN:
            if ev.button == 4: escala *= 1.2
            if ev.button == 5: escala /= 1.2
            if ev.button == 1: arrastando = True; pos_m = ev.pos
        if ev.type == pygame.MOUSEBUTTONUP: arrastando = False
        if ev.type == pygame.MOUSEMOTION and arrastando:
            offset_x += ev.pos[0] - pos_m[0]; offset_y += ev.pos[1] - pos_m[1]; pos_m = ev.pos

    scan_atual = processar_slam()

    # Desenha o Occupancy Grid
    for (gx, gy), info in list(pontos_mapa.items()):
        tipo, conf = info
        if tipo == 1 and conf < CONF_MIN_EXIBIR: continue
        
        sx, sy = world_to_screen(gx * GRID_SIZE, gy * GRID_SIZE)
        if -20 < sx < LARGURA+20 and -20 < sy < ALTURA+20:
            cor = COR_PAREDE if tipo == 1 else COR_BRANCO_LIVRE
            tamanho = max(1, int(GRID_SIZE * escala) + 1)
            pygame.draw.rect(tela, cor, (sx, sy, tamanho, tamanho))

    # Scan vivo
    centro = (LARGURA // 2 + offset_x, ALTURA // 2 + offset_y)
    for px, py in scan_atual:
        dx, dy = world_to_screen(px, py)
        pygame.draw.circle(tela, COR_SCAN_VIVO, (dx, dy), 1)

    # Robô Atlas
    pygame.draw.circle(tela, COR_ROBO, centro, 12, 3)
    pygame.draw.line(tela, (255, 255, 255), centro, (centro[0], centro[1]-20), 2)

    desenhar_hud_visao()

    # Telemetria
    f_info = pygame.font.SysFont("Consolas", 14)
    info_txts = [f"SLAM: OCCUPANCY GRID ACTIVE", f"ZONA BRANCA: AREA SEGURA", f"FPS: {int(relogio.get_fps())}"]
    for i, t in enumerate(info_txts):
        tela.blit(f_info.render(t, True, (0, 255, 100)), (20, ALTURA - 80 + i*18))

    pygame.display.flip()
    relogio.tick(60)
