
/**
 * \file
 * \brief 错误号相关定义头文件
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
 * \nswse 错误编码数据类型
 * @{
 */
typedef int error_t;    /* 兼容POSIX错误类型 */
typedef int sws_err_t;   /* Apollo错误类型 */
/** @} */

/**
 * \nswse POSIX错误值
 * @{
 */

#define SWS_OK              0       /**< \brief 操作成功            */
#define SWS_EPERM           1       /**< \brief 操作不允许          */
#define SWS_ENOENT          2       /**< \brief 文件或目录不存在    */
#define SWS_ESRCH           3       /**< \brief 进程不存在          */
#define SWS_EINTR           4       /**< \brief 调用被中断          */
#define SWS_EIO             5       /**< \brief I/O 错误            */
#define SWS_ENXIO           6       /**< \brief 设备或地址不存在    */
#define SWS_E2BIG           7       /**< \brief 参数列表太长        */
#define SWS_ENOEXEC         8       /**< \brief 可执行文件格式错误  */
#define SWS_EBADF           9       /**< \brief 文件描述符损坏      */
#define SWS_ECHILD          10      /**< \brief 没有子进程          */
#define SWS_EAGAIN          11      /**< \brief 资源不可用，需重试  */
#define SWS_ENOMEM          12      /**< \brief 空间（内存）不足    */
#define SWS_EACCES          13      /**< \brief 权限不够            */
#define SWS_EFAULT          14      /**< \brief 地址错误            */
#define SWS_ENOTEMPTY       15      /**< \brief 目录非空            */
#define SWS_EBUSY           16      /**< \brief 设备或资源忙        */
#define SWS_EEXIST          17      /**< \brief 文件已经存在        */
#define SWS_EXDEV           18      /**< \brief 跨设备连接          */
#define SWS_ENODEV          19      /**< \brief 设备不存在          */
#define SWS_ENOTDIR         20      /**< \brief 不是目录            */
#define SWS_EISDIR          21      /**< \brief 是目录              */
#define SWS_EINVAL          22      /**< \brief 无效参数            */
#define SWS_ENFILE          23      /**< \brief 系统打开文件太多，描述符不够用 */
#define SWS_EMFILE          24      /**< \brief 打开的文件太多      */
#define SWS_ENOTTY          25      /**< \brief 不合适的I/O控制操作 */
#define SWS_ENSWSETOOLONG   26      /**< \brief 文件名太长          */
#define SWS_EFBIG           27      /**< \brief 文件太大            */
#define SWS_ENOSPC          28      /**< \brief 设备剩余空间不足    */
#define SWS_ESPIPE          29      /**< \brief 无效的搜索（Invalid seek） */
#define SWS_EROFS           30      /**< \brief 文件系统只读        */
#define SWS_EMLINK          31      /**< \brief 链接太多            */
#define SWS_EPIPE           32      /**< \brief 损坏的管道          */
#define SWS_EDEADLK         33      /**< \brief 资源可能死锁        */
#define SWS_ENOLCK          34      /**< \brief 无可用（空闲）的锁  */
#define SWS_ENOTSUP         35      /**< \brief 不支持              */
#define SWS_EMSGSIZE        36      /**< \brief 消息太大            */

/** @} */

/**
 * \nswse ANSI错误值
 * @{
 */

#define SWS_EDOM            37      /**< \brief 数学函数参数超出范围 */
#define SWS_ERANGE          38      /**< \brief 数学函数计算结果太大 */
#define SWS_EILSEQ          39      /**< \brief 非法的字节顺序       */
 
/** @} */

/**
 * \nswse 网络参数错误相关
 * @{
 */

#define SWS_EDESTADDRREQ    40      /**< \brief 需要目标地址         */
#define SWS_EPROTOTYPE      41      /**< \brief socket协议类型错误   */
#define SWS_ENOPROTOOPT     42      /**< \brief 协议不可用           */
#define SWS_EPROTONOSUPPORT 43      /**< \brief 协议不支持           */
#define SWS_ESOCKTNOSUPPORT 44      /**< \brief Socket类型不支持     */
#define SWS_EOPNOTSUPP      45      /**< \brief socket不支持该操作   */
#define SWS_EPFNOSUPPORT    46      /**< \brief 协议族不支持         */
#define SWS_EAFNOSUPPORT    47      /**< \brief 地址簇不支持         */
#define SWS_EADDRINUSE      48      /**< \brief 地址已经被占用       */
#define SWS_EADDRNOTAVAIL   49      /**< \brief 地址不可用           */
#define SWS_ENOTSOCK        50      /**< \brief 被操作对象不是socket */

/** @} */

/**
 * \nswse 可选择实现的错误值
 * @{
 */

#define SWS_ENETUNREACH     51        /**< \brief 网络不可达         */
#define SWS_ENETRESET       52        /**< \brief 网络中断了连接     */
#define SWS_ECONNABORTED    53        /**< \brief 连接中断           */
#define SWS_ECONNRESET      54        /**< \brief 连接复位           */
#define SWS_ENOBUFS         55        /**< \brief 缓冲空间不足       */
#define SWS_EISCONN         56        /**< \brief Socket已经连接     */
#define SWS_ENOTCONN        57        /**< \brief Socket没有连接     */
#define SWS_ESHUTDOWN       58        /**< \brief Socket已经关闭，不能发送数据 */
#define SWS_ETOOMANYREFS    59        /**< \brief 引用太多，无法拼接 */
#define SWS_ETIMEDOUT       60        /**< \brief 连接超时           */
#define SWS_ECONNREFUSED    61        /**< \brief 连接被拒绝         */
#define SWS_ENETDOWN        62        /**< \brief 网络已经停止       */
#define SWS_ETXTBSY         63        /**< \brief 文本文件忙         */
#define SWS_ELOOP           64        /**< \brief 符号链接级数太多   */
#define SWS_EHOSTUNREACH    65        /**< \brief 主机不可达         */
#define SWS_ENOTBLK         66        /**< \brief 非块设备           */
#define SWS_EHOSTDOWN       67        /**< \brief 主机已经关闭       */

/** @} */

/**
 * \nswse 非阻塞和中断I/O错误值
 * @{
 */

#define SWS_EINPROGRESS     68         /**< \brief 操作正在进行中 */
#define SWS_EALREADY        69         /**< \brief 连接正被使用中 */

/* 70 */

#define SWS_EWOULDBLOCK     SWS_EAGAIN  /**< \brief 操作会阻塞（同EAGAIN） */
#define SWS_ENOSYS          71         /**< \brief 不支持的功能（功能未实现）*/
 
/** @} */

/**
 * \nswse 异步I/O错误值
 * @{
 */
#define SWS_ECANCELED       72         /**< \brief 操作已经取消 */

/* 73 */
/** @} */

/**
 * \nswse 流相关错误值
 * @{
 */
#define SWS_ENOSR           74        /**< \brief 没有流资源 */
#define SWS_ENOSTR          75        /**< \brief 不是流设备 */
#define SWS_EPROTO          76        /**< \brief 协议错误 */
#define SWS_EBADMSG         77        /**< \brief 损坏的消息 */
#define SWS_ENODATA         78        /**< \brief 流中无数据 */
#define SWS_ETIME           79        /**< \brief 流ioctl()超时 */
#define SWS_ENOMSG          80        /**< \brief 没有所需的消息类型 */
#define SWS_EUCLEAN         81        /**< \brief Structure需要清理 */
/** @} */                            

/**
* \nswse 其它自定义错误值
 * @{
 */
 
#define SWS_EFULL           100     /**< \brief 满               */ 
#define SWS_EEMPTY          101     /**< \brief 空               */
#define SWS_ERXOV           102     /**< \brief 接收溢出         */
#define SWS_ETXUR           103     /**< \brief 发送Underrun错误 */
#define SWS_ESSA            104     /**< \brief 从机断言         */
#define SWS_ESSD            105     /**< \brief 从机解除断言     */
#define SWS_EHIGH           106     /**< \brief 值过高           */
#define SWS_ELOW            107     /**< \brief 值过低           */

/** @} */

/** \brief 用户自定义错误起始值 */
#define SWS_ERRNO_USER_START 2000

/** \brief 用户自定义错误值 */
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
