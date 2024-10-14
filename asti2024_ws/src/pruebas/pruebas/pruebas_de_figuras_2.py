import cv2
import imutils

def detect(contour):
    """
    Función que, dado un contorno, retorna la forma geométrica más cercana con base al número de lados del perímetro del
    mismo.
    :param contour: Contorno del que inferiremos una figura geométrica.
    :return: Texto correspondiente a la figura geométrica identificada (TRIANGULO, CUADRADO, RECTANGULO, PENTAGONO o CIRCULO)
    """
    # Hallamos el perímetro (cerrado) del contorno.
    perimeter = cv2.arcLength(contour, True)

    # Aproximamos un polígono al contorno, con base a su perímetro.
    approximate = cv2.approxPolyDP(contour, .04 * perimeter, True)

    print(len(approximate))

    # Si el polígono aproximado tiene 3 lados, entonces es un triángulo.
    if len(approximate) == 3:
        shape = 'TRIANGULO'
    # Si el polígono aproximado tiene 4 lados, entonces puede ser o un cuadrado o un rectángulo.
    elif len(approximate) == 4:
        # Calculamos la relación de aspecto.
        x, y, w, h = cv2.boundingRect(approximate)
        aspect_ratio = w / float(h)

        # La figura será un cuadrado si la relación de aspecto está entre 95% y 105%, es decir, si todos los lados miden
        # más o menos lo mismo. En caso contrario, se trata de un rectángulo.
        shape = 'CUADRADO' if .95 <= aspect_ratio <= 1.05 else 'RECTANGULO'
    # Si el polígono aproximado tiene 5 lados, es un pentágono.
    elif len(approximate) == 5:
        shape = 'PENTAGONO'
    # Por defecto, asumiremos que cualquier polígono con 6 o más lados es un círculo.
    elif len(approximate) == 10:
        shape = 'estrella'
    else:
        shape = 'circulo'

    return shape


imagen = [
    "/home/jcrex/uji/teams-robotics/astic/Detector_caracteres/dynamixel.png",
    "/home/jcrex/Imágenes/figuras_geométricas.png",
    "/home/jcrex/Imágenes/dataset_estrellas/estrella.png"
    "/home/jcrex/Imágenes/dataset_estrellas/estrella_3.jpeg",
    "/home/jcrex/Imágenes/dataset_estrellas/estrella_5.png",
    "/home/jcrex/Imágenes/dataset_estrellas/estrella_6.jpeg",
    "/home/jcrex/Imágenes/dataset_estrellas/estrella_7.jpeg",
    "/home/jcrex/Imágenes/dataset_estrellas/estrella_8.jpeg",
    "/home/jcrex/Imágenes/dataset_estrellas/estrella_9.jpeg",
    "/home/jcrex/Imágenes/dataset_estrellas/estrella_4.jpeg",
    "/home/jcrex/Imágenes/dataset_estrellas/estrella_10.jpeg"
    ]

# Cargamos la imagen de entrada y la redimensionamos.
image = cv2.imread(imagen[9])
resized = imutils.resize(image, width=380)

# Calculamos la relación de proporción entre la imagen original y la redimensionada.
ratio = image.shape[0] / float(resized.shape[0])

# Convertimos la imagen a escala de grises, la difuminamos y la binarizamos.
gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
blurred = cv2.GaussianBlur(gray, (3, 3), 0)
thresholded = cv2.threshold(blurred, 230, 255, cv2.THRESH_BINARY)[1]
cv2.imshow("thresholded", thresholded)
cv2.waitKey(1)

# Invierte los colores de la imagen
inverted_image = cv2.bitwise_not(thresholded)
cv2.imshow("Inverted Image", inverted_image)
cv2.waitKey(1)

# Hallamos los contornos.
contours = cv2.findContours(inverted_image.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
contours = imutils.grab_contours(contours)

print(contours)

# Iteramos sobre cada contorno...
for contour in contours:
    # Calculamos los momentos del contorno para encontrar su centro.
    M = cv2.moments(contour)
    try:
        center_x = int((M['m10'] / M['m00']) * ratio)
        center_y = int((M['m01'] / M['m00']) * ratio)
    except ZeroDivisionError:
        continue

    # Determinamos la forma geométrica del contorno usando la función que definimos previamente.
    shape = detect(contour)
    # Ajustamos el contorno a la imagen original.
    contour = contour.astype('float') * ratio
    contour = contour.astype('int')

    # Dibujamos el contorno y lo etiquetamos con el nombre de la figura geométrica identificada.
    cv2.drawContours(image, [contour], -1, (0, 255, 0), 2)
    cv2.putText(image, shape, (center_x, center_y), cv2.FONT_HERSHEY_SIMPLEX, .7, (255, 0, 0), 2)

    # Mostramos el contorno en la imagen original.
    cv2.imshow('Imagen', image)
    cv2.waitKey(1)

cv2.waitKey(0)
cv2.destroyAllWindows()