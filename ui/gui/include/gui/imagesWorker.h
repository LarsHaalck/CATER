#ifndef GUI_IMAGES_WORKER_H
#define GUI_IMAGES_WORKER_H

#include <QObject>
#include <filesystem>


class ImagesWorker : public QObject
{
    Q_OBJECT

public:
    ImagesWorker(std::filesystem::path& imgFolder);


public slots:
    void doWork()
    {
        QString result;
        /* ... here is the expensive or blocking operation ... */
        emit resultReady(result);
    }

signals:
    void resultReady(const QString& result);
};

#endif // GUI_IMAGES_WORKER_H
