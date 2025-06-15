# IMPLÉMENTATION D'INTERACTION HOMME-ROBOT BASÉE SUR LA DÉTECTION D’ÉMOTIONS

Ce projet est un système complet développé pour permettre au robot NAO d’interagir avec des humains dans un environnement de simulation Webots et de réagir en temps réel aux émotions détectées.

## 📋 Vue d’ensemble du projet

Le système combine la vision par ordinateur, l’apprentissage automatique et la robotique pour permettre au robot NAO de :

- Détecter et suivre les humains
- Analyser les émotions faciales en temps réel
- Réagir de manière appropriée selon l’émotion détectée
- S’approcher physiquement et interagir


## 🗂️ Structure du projet

```
Implementation-Interaction-Homme-Robot-avec-Detection-Emotions/
├── datasets/                                    # Jeux de données et prétraitement
│   ├── CK+48/                                  # Jeu de données CK+ (test)
│   ├── facial_emotion_detection_dataset/       # Jeu de données de détection d’émotions faciales (test)
│   ├── facial_emotion_detection_dataset_cropped/ # Visages recadrés
│   └── preprocessing_facial_emotion_detection_dataset.ipynb # Réduction de taille du jeu FED
├── deepface_model/                             # Notebooks de test DeepFace
│   ├── deepface_model_with_ck_plus_dataset.ipynb # Test DeepFace + CK+
│   ├── deepface_model_with_facial_emotion_detection_dataset.ipynb # Test DeepFace + FED
│   ├── informations_ck_plus/                   # Résultats de test CK+
│   └── informations_facial_emotion_detection_dataset/ # Résultats de test FED
├── face_model/                                 # Notebooks de test modèle facial
│   ├── face_model_with_ck_plus_dataset.ipynb  # Test modèle facial + CK+
│   ├── face_model_with_facial_emotion_detection_dataset.ipynb # Test modèle facial + FED
│   ├── face_model.h5/                          # Modèles entraînés
│   ├── informations_ck_plus/                   # Résultats de test CK+
│   └── informations_facial_emotion_detection_dataset/ # Résultats de test FED
└── Webots_simulation/                          # Simulation Webots
    ├── controllers/                            # Codes de contrôle du robot
    │   ├── nao_controller/                     # Contrôleur du robot NAO (my_world.wbt)
    │   │   ├── nao_controller.py              # Logique principale du robot
    │   │   ├── human_scanner.py               # Détection et suivi des humains
    │   │   ├── start_emotion_gui.py           # GUI pour la reconnaissance des émotions
    │   │   └── yolov8n.pt                     # Modèle YOLO pour détection humaine
    │   ├── my_controller/                     # Contrôleur my_world - Copy.wbt
    │   │   ├── my_controller.py               # Logique principale
    │   │   ├── human_scanner.py               # Détection et suivi des humains
    │   │   ├── start_emotion_gui.py           # GUI pour reconnaissance des émotions
    │   │   └── yolov8n.pt                     # Modèle YOLO
    │   └── human_pose/                         # Détection de posture humaine
    ├── motions/                                # Fichiers de mouvements du robot
    │   ├── Forwards50.motion                   # Avancer
    │   ├── TurnLeft60.motion                   # Tourner à gauche
    │   ├── HandWave.motion                     # Saluer
    │   ├── OpenArms.motion                     # Ouvrir les bras
    │   └── ... (autres fichiers de mouvement)
    ├── protos/                                 # Fichiers proto Webots
    └── worlds/                                 # Mondes de simulation
        └── my_world - Copy.wbt                 # Environnement de simulation principal
        └── my_world.wbt                        # Environnement alternatif


```

## 🛠️ Composants du système

### Système de reconnaissance des émotions

- **DeepFace**: Analyse des émotions faciales en temps réel (modèle pré-entraîné)
- **CNN (Singh)**: Autre modèle d’analyse faciale (pré-entraîné)
- **Émotions supportées**: Joie, tristesse, colère, peur, surprise, dégoût, neutre

### 2. Système de Détection Humaine

- **YOLOv8**: Détection d’humains en temps réel
- **LiDAR**: Mesure de distance et localisation
- **Caméra **: GAnalyse visuelle et détection d’émotions

### 3. Système Comportemental du Robot

- **Approche adaptative**: Actions basées sur les émotions détectées
- **Interaction physique**: Câlins, saluts, éloignement
- **Navigation autonome**: Recherche et suivi de personnes

## 📦 Prérequis

### Configuration Système

- **İşletim Sistemi**: Windows 10/11, macOS, Linux
- **Python**: 3.8 ou supérieur
- **Webots**: R2023b ou supérieur
- **Caméra**: Caméra USB (pour l’analyse des émotions)

### Bibliothèques Python

```bash
# Dépendances principales
pip install deepface
pip install ultralytics
pip install opencv-python
pip install PyQt5
pip install tensorflow
pip install numpy
pip install matplotlib
pip install mtcnn
pip install pandas
pip install scikit-learn
pip install yolo
```

### Installation Webots

1. Téléchargez Webots depuis [le site officiel](https://cyberbotics.com/)
2. Assurez-vous d’avoir les licences nécessaires pour le modèle NAO

## 🚀 Étapes d’Installation

### 1. Installation du Projet

```bash
# Clonez le dépôt
git clone <repository-url>
cd Implementation-Interaction-Homme-Robot-avec-Detection-Emotions

# Créez un environnement virtuel (recommandé)
python -m venv robot_env
source robot_env/bin/activate  # Linux/macOS
# ou
robot_env\Scripts\activate     # Windows

# Installez les dépendances
pip install -r requirements.txt
```

### 2. Configuration de la Simulation Webots

```bash
# Lancez Webots
# Fichier > Ouvrir un monde > Sélectionnez le fichier my_world.wbt
# Vous verrez le robot NAO dans l’environnement de simulation
```

### 3. Configuration de la Caméra
- Assurez-vous que votre caméra USB est connectée à l’ordinateur
- Vérifiez que les pilotes de la caméra sont installés

## 🎮 Instructions d’Utilisation

### 1. Exécution Simple

```bash
# Ouvrez Webots et chargez le fichier my_world - Copy.wbt
# Cliquez sur le bouton Play pour démarrer la simulation
# Le robot commencera automatiquement à rechercher une personne
```

### 2. Flux du Système
1. **Recherche d'humain**: Le robot scanne les alentours pour détecter une personne
2. **Approche**: Lorsqu'une personne est détectée, le robot s'en approche à une distance sécurisée
3. **Analyse des émotions**: Analyse faciale via la caméra pendant 5 secondes
4. **Réaction**: Le robot effectue une action selon l’émotion détectée :
   - **Joie** → Salue et effectue un geste d'essuyage du front
   - **Tristesse/Neutre** → S'approche et lève les deux bras comme pour un câlin
   - **Colère/Peur/Surprise/Dégoût** → Recule rapidement

## 📊 Jeux de Données et Tests de Modèles

### Jeux de Données Utilisés

1. **CK+ Dataset**: Expressions faciales capturées en laboratoire (pour test uniquement)
2. **Facial Emotion Detection Dataset**: Expressions capturées dans des conditions réelles (pour test uniquement)

**Remarque**: Ces jeux de données ne sont pas utilisés pour l'entraînement des modèles, mais uniquement pour l’évaluation de leurs performances.

### Notebooks de Test de Modèle

```bash
# Tester le modèle DeepFace avec le jeu de données CK+
jupyter notebook deepface_model/deepface_model_with_ck_plus_dataset.ipynb

# Tester le modèle DeepFace avec le jeu de données FED
jupyter notebook deepface_model/deepface_model_with_facial_emotion_detection_dataset.ipynb

# Tester le modèle facial personnalisé avec CK+
jupyter notebook face_model/face_model_with_ck_plus_dataset.ipynb

# Tester le modèle facial personnalisé avec FED
jupyter notebook face_model/face_model_with_facial_emotion_detection_dataset.ipynb
```

### Prétraitement des Données
```bash
# Réduction de la taille des images du jeu FED
jupyter notebook datasets/preprocessing_facial_emotion_detection_dataset.ipynb
```

## 🔧 Configuration

### Paramètres Comportementaux du Robot

Modifiables dans le fichier `nao_controller.py` :

- `TIME_STEP`: Intervalle de simulation (32ms)
- `control`: Nombre de cycles d’interaction (3)
- Durée d’analyse des émotions (par défaut : 5 secondes)


### 📈 Optimisation des Performances

### Configuration Matérielle Recommandée

- **RAM**: Minimum 8GB (16Go recommandé)
- **GPU**: Compatible CUDA (facultatif mais recommandé)
- **CPU**: Intel i5 ou supérieur recommandé


_Son güncelleme: 2025_
