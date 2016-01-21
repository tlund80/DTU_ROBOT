/*
 * ImgAnnotation.h
 *
 *  Created on: Dec 12, 2011
 *      Author: RAHA
 */

#ifndef IMGANNOTATION_H_
#define IMGANNOTATION_H_

#include "ui_AnnotateDialog.hpp"
#include "SharedData.hpp"
#include <QMouseEvent>
#include <QPaintEvent>
//#include <QtGui>

namespace dti {

class ImgAnnotation  : public QDialog
{
    Q_OBJECT

public:
	ImgAnnotation(QWidget *parent, SharedData *shared, QString title);
	virtual ~ImgAnnotation();


    void 	updateImage(QImage imgQ);
    QRect 	selectRoi();
    QRect 	getRoi();
    Ui::annotationDialog *getAnnotationDialog() {return _dialog;}
private:
    void 	updatePixmap(QPixmap pix, int width, int height);
    void 	setMouseCursorPos(QPoint);
    QPoint 	getMouseCursorPos();

    SharedData *_sharedData;
    enum AnnotateState {
    	nothingSelected,
    	startSelected,
    	endSelected,
    	cancel,
    	finish
    } AnnotateState;

    QPixmap _pixOrg;
    QSize _orginal_img_size;
    double _scale_w;
    double _scale_h;

    // mouse data
    QRect _selectionBox;
    bool _leftButtonRelease;
    bool _rightButtonRelease;

    QPoint _startPoint;
    QPoint _mouseCursorPos;
    bool _drawBox;
    QRect *_pbox;
    Ui::annotationDialog *_dialog;
protected:
    void mouseMoveEvent(QMouseEvent *ev);
    void mousePressEvent(QMouseEvent *ev);
    void mouseReleaseEvent(QMouseEvent *ev);
    void paintEvent(QPaintEvent *ev);
    void resizeEvent(QResizeEvent *ev);
//signals:
Q_SIGNALS:
//    void newMouseData(struct MouseData mouseData);
//    void onMouseMoveEvent();

};

} /* namespace dti */
#endif /* IMGANNOTATION_H_ */
