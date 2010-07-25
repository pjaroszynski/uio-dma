#include <linux/device.h>
#include <linux/module.h>
#include <linux/pci.h>

#include "uio-dma.h"

#define DRIVER_VERSION  "0.0.1"
#define DRIVER_AUTHOR   "Piotr Jaroszyński <p.jaroszynski@gmail.com>"
#define DRIVER_DESC     "Driver enabling UIO-DMA for a PCI device"

struct dev_data {
	/** UIO-DMA device id */
	uint32_t uio_dma_id;
};

ssize_t uio_dma_id_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct dev_data *dd = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "0x%08x\n", dd->uio_dma_id);
}

/** Device attribute with the UIO-DMA device id */
static DEVICE_ATTR(uio_dma_id, S_IRUGO, uio_dma_id_show, NULL);

static int __devinit probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	int err;
	struct dev_data *dd;

	err = pci_enable_device(pdev);
	if (err) {
		dev_err(&pdev->dev, "%s: pci_enable_device failed: %d\n", __func__, err);
		return err;
	}

	dd = kmalloc(sizeof(*dd), GFP_KERNEL);

	if (! dd) {
		err = -ENOMEM;
		goto err_kmalloc;
	}

	pci_set_drvdata(pdev, dd);

	err = uio_dma_device_open(&pdev->dev, &dd->uio_dma_id);
	if (err)
		goto err_uio_dma;

	err = device_create_file(&pdev->dev, &dev_attr_uio_dma_id);
	if (err)
		goto err_create_file;

	return 0;

err_create_file:
	uio_dma_device_close(dd->uio_dma_id);
err_uio_dma:
	kfree(dd);
err_kmalloc:
	pci_disable_device(pdev);

	return err;
}

static void remove(struct pci_dev *pdev)
{
	struct dev_data *dd = pci_get_drvdata(pdev);

	device_remove_file(&pdev->dev, &dev_attr_uio_dma_id);
	uio_dma_device_close(dd->uio_dma_id);
	kfree(dd);
	pci_disable_device(pdev);
}

static struct pci_driver driver = {
	.name = "uio-dma-pci",
	.id_table = NULL, /* only dynamic id's */
	.probe = probe,
	.remove = remove,
};

static int __init init(void)
{
	pr_info(DRIVER_DESC " version: " DRIVER_VERSION "\n");
	return pci_register_driver(&driver);
}

static void __exit cleanup(void)
{
	pci_unregister_driver(&driver);
}

module_init(init);
module_exit(cleanup);

MODULE_VERSION(DRIVER_VERSION);
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
