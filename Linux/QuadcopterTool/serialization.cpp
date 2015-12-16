/*
    Copyright 2013 - 2015 Benjamin Vedder	benjamin@vedder.se

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#include "serialization.h"
#include <QFileDialog>
#include <qxmlstream.h>
#include <QFile>
#include <QDebug>

Serialization::Serialization(QObject *parent) :
    QObject(parent)
{
}

bool Serialization::writeAncorConf(const QVector<ANCHOR_SETTINGS_t> &anch_set, const MAP_LIMITS_t &map_lim, QWidget *parent)
{
    QString filename = QFileDialog::getSaveFileName(parent,
                                                    tr("Save Configuration"), ".",
                                                    tr("Xml files (*.xml)"));

    if (!filename.toLower().endsWith(".xml")) {
        filename.append(".xml");
    }

    QFile file(filename);
    if (!file.open(QIODevice::WriteOnly)) {
        return false;
    }

    QXmlStreamWriter xmlwriter(&file);
    xmlwriter.setAutoFormatting(true);
    xmlwriter.writeStartDocument();

    xmlwriter.writeStartElement("MapConfiguration");

    for (int i = 0;i < anch_set.size();i++) {
        const ANCHOR_SETTINGS_t *anch = &anch_set[i];
        xmlwriter.writeStartElement("Anchor");
        xmlwriter.writeTextElement("id", QString::number(anch->id));
        xmlwriter.writeTextElement("px", QString::number(anch->px));
        xmlwriter.writeTextElement("py", QString::number(anch->py));
        xmlwriter.writeTextElement("pz", QString::number(anch->pz));
        xmlwriter.writeEndElement();
    }

    xmlwriter.writeTextElement("min_x", QString::number(map_lim.min_x));
    xmlwriter.writeTextElement("max_x", QString::number(map_lim.max_x));
    xmlwriter.writeTextElement("min_y", QString::number(map_lim.min_y));
    xmlwriter.writeTextElement("max_y", QString::number(map_lim.max_y));

    xmlwriter.writeEndElement();
    xmlwriter.writeEndDocument();
    file.close();

    return true;
}

bool Serialization::readAncorConf(QVector<ANCHOR_SETTINGS_t> &anch_set, MAP_LIMITS_t &map_lim, QWidget *parent)
{
    QString filename = QFileDialog::getOpenFileName(parent,
                                                    tr("Load Configuration"), ".",
                                                    tr("Xml files (*.xml)"));

    bool retval = true;

    QFile file(filename);
    if (!file.open(QIODevice::ReadOnly)) {
        return false;
    }

    anch_set.clear();

    QXmlStreamReader xmlreader(&file);
    xmlreader.readNext();
    while (!xmlreader.atEnd()) {
        if (xmlreader.isStartElement()) {
            if (xmlreader.name() == "MapConfiguration") {
                xmlreader.readNext();
                while (!xmlreader.atEnd()) {
                    if (xmlreader.isEndElement()) {
                        xmlreader.readNext();
                        break;
                    }

                    if(xmlreader.name() == "min_x") {map_lim.min_x = xmlreader.readElementText().toDouble();}
                    else if (xmlreader.name() == "max_x") {map_lim.max_x = xmlreader.readElementText().toDouble();}
                    else if (xmlreader.name() == "min_y") {map_lim.min_y = xmlreader.readElementText().toDouble();}
                    else if (xmlreader.name() == "max_y") {map_lim.max_y = xmlreader.readElementText().toDouble();}

                    if (xmlreader.isStartElement()) {
                        if (xmlreader.name() == "Anchor") {
                            xmlreader.readNext();

                            anch_set.resize(anch_set.size() + 1);
                            ANCHOR_SETTINGS_t &anch = anch_set[anch_set.size() - 1];

                            while (!xmlreader.atEnd()) {
                                if (xmlreader.isEndElement()) {
                                    xmlreader.readNext();
                                    break;
                                }

                                if(xmlreader.name() == "id") {anch.id = xmlreader.readElementText().toInt();}
                                else if (xmlreader.name() == "px") {anch.px = xmlreader.readElementText().toDouble();}
                                else if (xmlreader.name() == "py") {anch.py = xmlreader.readElementText().toDouble();}
                                else if (xmlreader.name() == "pz") {anch.pz = xmlreader.readElementText().toDouble();}
                                xmlreader.readNext();
                            }
                        }
                    }

                    xmlreader.readNext();
                }
            } else {
                retval = false;
                break;
            }
        } else {
            xmlreader.readNext();
        }
    }

    file.close();

    return retval;
}
