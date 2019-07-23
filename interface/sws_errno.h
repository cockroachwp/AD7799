
/**
 * \file
 * \brief �������ض���ͷ�ļ�
 *
 * \internal
 * \par Modification history
 * - 1.00 14-11-01  zen, first implementation.
 * \endinternal
 */

#ifndef __SWS_ERRNO_H
#define __SWS_ERRNO_H
 
#ifdef __cplusplus
extern "C" {
#endif


/**
 * \addtogroup sws_if_errno
 * \copydoc sws_errno.h
 * @{
 */
 
/**
 * \nswse ���������������
 * @{
 */
typedef int error_t;    /* ����POSIX�������� */
typedef int sws_err_t;   /* Apollo�������� */
/** @} */

/**
 * \nswse POSIX����ֵ
 * @{
 */

#define SWS_OK              0       /**< \brief �����ɹ�            */
#define SWS_EPERM           1       /**< \brief ����������          */
#define SWS_ENOENT          2       /**< \brief �ļ���Ŀ¼������    */
#define SWS_ESRCH           3       /**< \brief ���̲�����          */
#define SWS_EINTR           4       /**< \brief ���ñ��ж�          */
#define SWS_EIO             5       /**< \brief I/O ����            */
#define SWS_ENXIO           6       /**< \brief �豸���ַ������    */
#define SWS_E2BIG           7       /**< \brief �����б�̫��        */
#define SWS_ENOEXEC         8       /**< \brief ��ִ���ļ���ʽ����  */
#define SWS_EBADF           9       /**< \brief �ļ���������      */
#define SWS_ECHILD          10      /**< \brief û���ӽ���          */
#define SWS_EAGAIN          11      /**< \brief ��Դ�����ã�������  */
#define SWS_ENOMEM          12      /**< \brief �ռ䣨�ڴ棩����    */
#define SWS_EACCES          13      /**< \brief Ȩ�޲���            */
#define SWS_EFAULT          14      /**< \brief ��ַ����            */
#define SWS_ENOTEMPTY       15      /**< \brief Ŀ¼�ǿ�            */
#define SWS_EBUSY           16      /**< \brief �豸����Դæ        */
#define SWS_EEXIST          17      /**< \brief �ļ��Ѿ�����        */
#define SWS_EXDEV           18      /**< \brief ���豸����          */
#define SWS_ENODEV          19      /**< \brief �豸������          */
#define SWS_ENOTDIR         20      /**< \brief ����Ŀ¼            */
#define SWS_EISDIR          21      /**< \brief ��Ŀ¼              */
#define SWS_EINVAL          22      /**< \brief ��Ч����            */
#define SWS_ENFILE          23      /**< \brief ϵͳ���ļ�̫�࣬������������ */
#define SWS_EMFILE          24      /**< \brief �򿪵��ļ�̫��      */
#define SWS_ENOTTY          25      /**< \brief �����ʵ�I/O���Ʋ��� */
#define SWS_ENSWSETOOLONG   26      /**< \brief �ļ���̫��          */
#define SWS_EFBIG           27      /**< \brief �ļ�̫��            */
#define SWS_ENOSPC          28      /**< \brief �豸ʣ��ռ䲻��    */
#define SWS_ESPIPE          29      /**< \brief ��Ч��������Invalid seek�� */
#define SWS_EROFS           30      /**< \brief �ļ�ϵͳֻ��        */
#define SWS_EMLINK          31      /**< \brief ����̫��            */
#define SWS_EPIPE           32      /**< \brief �𻵵Ĺܵ�          */
#define SWS_EDEADLK         33      /**< \brief ��Դ��������        */
#define SWS_ENOLCK          34      /**< \brief �޿��ã����У�����  */
#define SWS_ENOTSUP         35      /**< \brief ��֧��              */
#define SWS_EMSGSIZE        36      /**< \brief ��Ϣ̫��            */

/** @} */

/**
 * \nswse ANSI����ֵ
 * @{
 */

#define SWS_EDOM            37      /**< \brief ��ѧ��������������Χ */
#define SWS_ERANGE          38      /**< \brief ��ѧ����������̫�� */
#define SWS_EILSEQ          39      /**< \brief �Ƿ����ֽ�˳��       */
 
/** @} */

/**
 * \nswse ��������������
 * @{
 */

#define SWS_EDESTADDRREQ    40      /**< \brief ��ҪĿ���ַ         */
#define SWS_EPROTOTYPE      41      /**< \brief socketЭ�����ʹ���   */
#define SWS_ENOPROTOOPT     42      /**< \brief Э�鲻����           */
#define SWS_EPROTONOSUPPORT 43      /**< \brief Э�鲻֧��           */
#define SWS_ESOCKTNOSUPPORT 44      /**< \brief Socket���Ͳ�֧��     */
#define SWS_EOPNOTSUPP      45      /**< \brief socket��֧�ָò���   */
#define SWS_EPFNOSUPPORT    46      /**< \brief Э���岻֧��         */
#define SWS_EAFNOSUPPORT    47      /**< \brief ��ַ�ز�֧��         */
#define SWS_EADDRINUSE      48      /**< \brief ��ַ�Ѿ���ռ��       */
#define SWS_EADDRNOTAVAIL   49      /**< \brief ��ַ������           */
#define SWS_ENOTSOCK        50      /**< \brief ������������socket */

/** @} */

/**
 * \nswse ��ѡ��ʵ�ֵĴ���ֵ
 * @{
 */

#define SWS_ENETUNREACH     51        /**< \brief ���粻�ɴ�         */
#define SWS_ENETRESET       52        /**< \brief �����ж�������     */
#define SWS_ECONNABORTED    53        /**< \brief �����ж�           */
#define SWS_ECONNRESET      54        /**< \brief ���Ӹ�λ           */
#define SWS_ENOBUFS         55        /**< \brief ����ռ䲻��       */
#define SWS_EISCONN         56        /**< \brief Socket�Ѿ�����     */
#define SWS_ENOTCONN        57        /**< \brief Socketû������     */
#define SWS_ESHUTDOWN       58        /**< \brief Socket�Ѿ��رգ����ܷ������� */
#define SWS_ETOOMANYREFS    59        /**< \brief ����̫�࣬�޷�ƴ�� */
#define SWS_ETIMEDOUT       60        /**< \brief ���ӳ�ʱ           */
#define SWS_ECONNREFUSED    61        /**< \brief ���ӱ��ܾ�         */
#define SWS_ENETDOWN        62        /**< \brief �����Ѿ�ֹͣ       */
#define SWS_ETXTBSY         63        /**< \brief �ı��ļ�æ         */
#define SWS_ELOOP           64        /**< \brief �������Ӽ���̫��   */
#define SWS_EHOSTUNREACH    65        /**< \brief �������ɴ�         */
#define SWS_ENOTBLK         66        /**< \brief �ǿ��豸           */
#define SWS_EHOSTDOWN       67        /**< \brief �����Ѿ��ر�       */

/** @} */

/**
 * \nswse ���������ж�I/O����ֵ
 * @{
 */

#define SWS_EINPROGRESS     68         /**< \brief �������ڽ����� */
#define SWS_EALREADY        69         /**< \brief ��������ʹ���� */

/* 70 */

#define SWS_EWOULDBLOCK     SWS_EAGAIN  /**< \brief ������������ͬEAGAIN�� */
#define SWS_ENOSYS          71         /**< \brief ��֧�ֵĹ��ܣ�����δʵ�֣�*/
 
/** @} */

/**
 * \nswse �첽I/O����ֵ
 * @{
 */
#define SWS_ECANCELED       72         /**< \brief �����Ѿ�ȡ�� */

/* 73 */
/** @} */

/**
 * \nswse ����ش���ֵ
 * @{
 */
#define SWS_ENOSR           74        /**< \brief û������Դ */
#define SWS_ENOSTR          75        /**< \brief �������豸 */
#define SWS_EPROTO          76        /**< \brief Э����� */
#define SWS_EBADMSG         77        /**< \brief �𻵵���Ϣ */
#define SWS_ENODATA         78        /**< \brief ���������� */
#define SWS_ETIME           79        /**< \brief ��ioctl()��ʱ */
#define SWS_ENOMSG          80        /**< \brief û���������Ϣ���� */
#define SWS_EUCLEAN         81        /**< \brief Structure��Ҫ���� */
/** @} */                            

/**
* \nswse �����Զ������ֵ
 * @{
 */
 
#define SWS_EFULL           100     /**< \brief ��               */ 
#define SWS_EEMPTY          101     /**< \brief ��               */
#define SWS_ERXOV           102     /**< \brief �������         */
#define SWS_ETXUR           103     /**< \brief ����Underrun���� */
#define SWS_ESSA            104     /**< \brief �ӻ�����         */
#define SWS_ESSD            105     /**< \brief �ӻ��������     */
#define SWS_EHIGH           106     /**< \brief ֵ����           */
#define SWS_ELOW            107     /**< \brief ֵ����           */

/** @} */

/** \brief �û��Զ��������ʼֵ */
#define SWS_ERRNO_USER_START 2000

/** \brief �û��Զ������ֵ */
#define SWS_ERRNO_USER(x)    (SWS_ERRNO_USER_START + (x))

/** @} */

/**
 * @} 
 */

#ifdef __cplusplus
}
#endif

#endif /* __SWS_ERRNO_H */

/* end of file */
