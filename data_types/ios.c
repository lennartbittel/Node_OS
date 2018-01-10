#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include<linux/slab.h>                 //kmalloc()
#include<linux/uaccess.h>              //copy_to/from_user()
#include <linux/ioctl.h>
#include <linux/assoc_array.h>
#include <linux/list.h>
#include <linux/rbtree.h>
struct dict_ent {
	struct list_head list;
	int key;
	int val;
};
 



struct unode{
       	struct rb_node __rb_node;
	int __num;      //this is to record our data.
	int pointer;
};

struct unode * rb_search_unode( struct rb_root * root , int target ){

       struct rb_node * n = root->rb_node;
       struct unode * ans;

       while( n ){
              //Get the parent struct to obtain the data for comparison
              ans = rb_entry( n , struct unode , __rb_node );

              if( target < ans->__num )
                     n = n->rb_left;
              else if( target > ans->__num )
                     n = n->rb_right;
              else
                     return ans;

       }
       return NULL;

}
struct unode * rb_insert_unode( struct rb_root * root , int target , struct rb_node * source ){

       struct rb_node **p = &root->rb_node;
       struct rb_node *parent = NULL;
       Ö * ans;

       while( *p ){

              parent = *p;
              ans = rb_entry( parent , struct unode , __rb_node );

              if( target < ans->__num )
                     p = &(*p)->rb_left;
              else if( target > ans->__num )
                     p = &(*p)->rb_right;
              else
                     return ans;

       }
       rb_link_node( source , parent , p );             //Insert this new node as a red leaf.
       rb_insert_color( source , root );           //Rebalance the tree, finish inserting
       return NULL;

}

void rb_erase_unode( struct rb_node * source , struct rb_root * root ){

       struct unode * target;
      
       target = rb_entry( source , struct unode , __rb_node );
       rb_erase( source , root );                           //Erase the node
       kfree( target );                                     //Free the memory

}

static int __init etx_driver_init(void)
{
	/*printk("hello\n");
	struct list_head ex;
	struct  dict_ent d;
	d.key=1;
	d.val=3;
	INIT_LIST_HEAD(&ex);
	list_add(&(d.list),&ex);*/
	struct rb_root root = RB_ROOT;
	struct unode * node;
	struct unode * res;
	int i;
	int op=42;
	for(i=0;i<100;++i)
	{
		node = ( struct unode * )kmalloc( sizeof( struct unode ),GFP_KERNEL );
		rb_insert_unode( &root , i, &node->__rb_node );
		node->__num = i;
		node->pointer=i*i;
	}
	res=rb_search_unode( &root ,op);
	printk("res:%d,%d\n", res->__num,res-> pointer);
   	return 0;
}
 
void __exit etx_driver_exit(void)
{
	printk("Bye\n");
}
 
module_init(etx_driver_init);
module_exit(etx_driver_exit);
 
MODULE_LICENSE("GPL");
MODULE_AUTHOR("EmbeTronicX <embetronicx@gmail.com or admin@embetronicx.com>");
MODULE_DESCRIPTION("A simple device driver");
MODULE_VERSION("1.5");
