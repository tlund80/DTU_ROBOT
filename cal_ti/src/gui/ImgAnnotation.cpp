/*
 * ImgAnnotation.cpp
 *
 *  Created on: Dec 12, 2011
 *      Author: RAHA
 */

#include "../../include/cal_ti/ImgAnnotation.hpp"
#include <iostream>
#include <qpainter.h>
#include <qlabel.h>
namespace dti {


ImgAnnotation::ImgAnnotation(QWidget *parent, SharedData* shared, QString title) : QDialog(parent), _scale_w(1), _scale_h(1)
{
	_sharedData = shared;
	// Initialize variables
	_startPoint.setX(0);
	_startPoint.setY(0);
	_mouseCursorPos.setX(0);
	_mouseCursorPos.setY(0);
	_drawBox = false;
	// Initialize MouseData structure
	_leftButtonRelease  = false;
	_rightButtonRelease = false;
	_selectionBox.setWidth(0);
	_selectionBox.setHeight(0);
	_selectionBox.setX(0);
	_selectionBox.setY(0);

	_pbox 	= new QRect(0, 0, 0, 0);
	_dialog = new Ui::annotationDialog();
	_dialog->setupUi(this);

    this->setVisible(true);
	this->setWindowTitle(title);
} // ImgAnnotation constructor

ImgAnnotation::~ImgAnnotation()
{

}

void ImgAnnotation::updatePixmap(QPixmap pix, int width, int height)
{
  //  _dialog->_imgLabel->setScaledContents(true);
    _dialog->_imgLabel->setPixmap(pix.scaled(QSize(width,height),Qt::KeepAspectRatio));

    QSize s = _dialog->_imgLabel->pixmap()->size();

    _scale_w = double(_orginal_img_size.width())/double(s.width());
    _scale_h = double(_orginal_img_size.height())/double(s.height());

}

void ImgAnnotation::updateImage(QImage imgQ)
{
	_pixOrg = QPixmap::fromImage(imgQ);

    //Save orginal image size
    _orginal_img_size.setHeight(_pixOrg.height());
    _orginal_img_size.setWidth(_pixOrg.width());

    updatePixmap(_pixOrg,_dialog->_annotationDialog->width(), _dialog->_annotationDialog->height());
}

QRect ImgAnnotation::getRoi()
{
    // Stop drawing box
    _drawBox = false;
	return _selectionBox;
}

void ImgAnnotation::setMouseCursorPos(QPoint input)
{
    _mouseCursorPos=input;
} // setMouseCursorPos()

QPoint ImgAnnotation::getMouseCursorPos()
{
    return _mouseCursorPos;
} // getMouseXPos()

void ImgAnnotation::mouseMoveEvent(QMouseEvent *ev)
{
    // Save mouse cursor position
    setMouseCursorPos(ev->pos());
    // Update box width and height if box drawing is in progress
    if(_drawBox)
    {
        _pbox->setWidth(double(getMouseCursorPos().x())*_scale_w  - _startPoint.x());
        _pbox->setHeight(double(getMouseCursorPos().y())*_scale_h - _startPoint.y());
    }

    // Inform main window of mouse move event
    this->update();

} // mouseMoveEvent()

void ImgAnnotation::mouseReleaseEvent(QMouseEvent *ev)
{
    // Update cursor position
    setMouseCursorPos(ev->pos());
    // On left mouse button release
    if(ev->button()==Qt::LeftButton)
    {
        // Set leftButtonRelease flag to TRUE
        _leftButtonRelease = true;
        if(_drawBox)
        {
            // Save box dimensions
            _selectionBox.setX(_pbox->left());
            _selectionBox.setY(_pbox->top());
            _selectionBox.setWidth(_pbox->width());
            _selectionBox.setHeight(_pbox->height());
            // Set leftButtonRelease flag to TRUE
            _leftButtonRelease = true;
            // Inform main window of event

        }
        // Set leftButtonRelease flag to FALSE
        _leftButtonRelease = false;
    }
    // On right mouse button release
    else if (ev->button() == Qt::RightButton)
    {
        // If user presses (and then releases) the right mouse button while drawing box, stop drawing box
        if(_drawBox)
            _drawBox = false;
        else
        {
            // Set rightButtonRelease flag to TRUE
			_rightButtonRelease = true;
            // Inform main window of event
//            emit newMouseData(mouseData);
            // Set rightButtonRelease flag to FALSE
            _rightButtonRelease = false;
        }
    }
    repaint();
} // mouseReleaseEvent()

void ImgAnnotation::mousePressEvent(QMouseEvent *ev)
{
    // Update cursor position
    setMouseCursorPos(ev->pos());;
    if(ev->button()==Qt::LeftButton)
    {
        // Start drawing box
        _startPoint = ev->pos();
        _pbox = new QRect(double(_startPoint.x())*_scale_w, double(_startPoint.y())*_scale_h, 0, 0);
		_drawBox = true;
	}
    repaint();
} // mousePressEvent()

void ImgAnnotation::paintEvent(QPaintEvent *ev)
{
    QDialog::paintEvent(ev);
	QPixmap pix = _pixOrg;//(QPixmap*)_dialog->_imgLabel->pixmap();
    QPainter painter(&pix);
    // Draw box
    if(_drawBox)
    {
        QPen p; p.setWidth(8); p.setColor(Qt::blue);
        painter.setPen(p);
        painter.setBrush(Qt::NoBrush);

        painter.drawRect(*_pbox);
    }
    updatePixmap(pix, _dialog->_annotationDialog->width(), _dialog->_annotationDialog->height());
//    update();
} // paintEvent()


void ImgAnnotation::resizeEvent(QResizeEvent *ev){
 //   if(QResizeEvent::User)
  //    _dialog->_imgLabel->resize(ev->size());
}


} /* namespace dti */
